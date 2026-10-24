param([string]$Compiler = 'gcc')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$root = Split-Path $PSScriptRoot
$state = Get-HostSourceSnapshot -ProjectRoot $root
$build=Join-Path $root 'Tests/group_only_stand_build'
$null=New-Item -ItemType Directory -Path $build -Force
$base=@('-std=c99','-Wall','-Wextra','-Werror','-pedantic',
 '-I',(Join-Path $root 'Tests/host_stubs'),'-I',(Join-Path $root 'Tests'),
 '-I',(Join-Path $root 'Drivers/camera'),'-I',(Join-Path $root 'User/motion'),'-I',(Join-Path $root 'User/servo'),
 (Join-Path $root 'Tests/test_group_only_stand.c'),(Join-Path $root 'Drivers/camera/camera.c'),
 (Join-Path $root 'User/motion/robot_control.c'),(Join-Path $root 'User/motion/robot_walk_vision.c'),(Join-Path $root 'User/motion/motion_player.c'))
$groups=Join-Path $root 'User/motion/motion_groups.c'
$source=[IO.File]::ReadAllText($groups)
$match=[regex]::Match($source,'(?s)(static const MotionFrame_t s_qrHeadShake\[\]\s*=\s*\{)(.*?)(\r?\n\};)')
if(!$match.Success){throw 'Head fixture missing'}
$body=$match.Groups[2].Value
$last=[regex]::Match($body,'\{\s*400\s*,\s*\{([^}]*)\}\s*\}')
if(!$last.Success){throw 'Head final frame missing'}
$angles=@($last.Groups[1].Value.Split(',')|ForEach-Object{$_.Trim()})
$angles[12]='1'
$newBody=$body.Substring(0,$last.Index)+'{ 400, { '+($angles -join ', ')+' } }'+$body.Substring($last.Index+$last.Length)
$source=$source.Substring(0,$match.Groups[2].Index)+$newBody+$source.Substring($match.Groups[2].Index+$body.Length)
$fixture=Join-Path $build 'nonzero_head_fixture.c'
[IO.File]::WriteAllText($fixture,$source,[Text.UTF8Encoding]::new($false))
$cases=@(
 @{Name='full_ascii';Defs=@('-DROBOT_RUN_MODE=3U');Groups=$groups},
 @{Name='full_binary';Defs=@('-DROBOT_RUN_MODE=3U','-DCAMERA_PROTOCOL_BINARY=1U');Groups=$groups},
 @{Name='image_ascii';Defs=@('-DROBOT_RUN_MODE=2U');Groups=$groups},
 @{Name='image_binary';Defs=@('-DROBOT_RUN_MODE=2U','-DCAMERA_PROTOCOL_BINARY=1U');Groups=$groups},
 @{Name='nonzero_end_recovers';Defs=@('-DROBOT_RUN_MODE=3U','-DTEST_NONZERO_SHAPE_END=1');Groups=$fixture},
 @{Name='legacy_recovery_wait';Defs=@('-DROBOT_RUN_MODE=3U','-DROBOT_SKIP_DUPLICATE_SHAPE_STAND=0U','-DROBOT_WALK_SETTLE_MS=300U','-DROBOT_WALK_STARTUP_SETTLE_MS=300U');Groups=$groups}
)
$taskSavedPath=$env:PATH
try{
 $env:PATH=(Split-Path $compiler)+[IO.Path]::PathSeparator+$env:PATH
 foreach($case in $cases){
  $binary=Join-Path $build ($case.Name+'.exe')
  & $compiler @base @($case.Defs) $case.Groups '-o' $binary
  if($LASTEXITCODE -ne 0){throw ('Compile failed: '+$case.Name)}
  & $binary | Tee-Object -FilePath (Join-Path $build ($case.Name+'.log'))
  if($LASTEXITCODE -ne 0){throw ('Test failed: '+$case.Name)}
  Write-Output ('CASE PASSED: '+$case.Name)
 }
 foreach($property in $state.protected_sha256.PSObject.Properties){
  $actual=(Get-FileHash -LiteralPath (Join-Path $root $property.Name) -Algorithm SHA256).Hash
  if($actual.ToLowerInvariant() -ne $property.Value){throw ('Protected file changed: '+$property.Name)}
 }
 Write-Output 'ALL 6 RECOVERY CONFIGURATIONS PASSED; GROUPS AND SERVO SOURCES UNCHANGED'
}finally{$env:PATH=$taskSavedPath}
