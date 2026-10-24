param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$root = Split-Path $PSScriptRoot
$state = Get-HostSourceSnapshot -ProjectRoot $root
$build = Join-Path $root 'Tests/left_turn_settle_build'
$null = New-Item -ItemType Directory -Path $build -Force
$taskSavedPath = $env:PATH
$base = @('-std=c99','-Wall','-Wextra','-Werror','-pedantic',
    '-I',(Join-Path $root 'Tests/host_stubs'),'-I',(Join-Path $root 'Tests'),
    '-I',(Join-Path $root 'Drivers/camera'),'-I',(Join-Path $root 'User/motion'),'-I',(Join-Path $root 'User/servo'),
    (Join-Path $root 'Tests/test_left_turn_settle.c'),(Join-Path $root 'Drivers/camera/camera.c'),
    (Join-Path $root 'User/motion/robot_control.c'),(Join-Path $root 'User/motion/robot_walk_vision.c'),
    (Join-Path $root 'User/motion/motion_player.c'),(Join-Path $root 'User/motion/motion_groups.c'))
$cases=@(
    @{Name='full_ascii'; Defines=@('-DROBOT_RUN_MODE=3U')},
    @{Name='full_binary'; Defines=@('-DROBOT_RUN_MODE=3U','-DCAMERA_PROTOCOL_BINARY=1U')},
    @{Name='line_ascii'; Defines=@('-DROBOT_RUN_MODE=4U')},
    @{Name='line_binary'; Defines=@('-DROBOT_RUN_MODE=4U','-DCAMERA_PROTOCOL_BINARY=1U')},
    @{Name='100ms_override'; Defines=@('-DROBOT_RUN_MODE=3U','-DROBOT_WALK_TURN_SETTLE_MS=100U')},
    @{Name='original_300ms_override'; Defines=@('-DROBOT_RUN_MODE=3U','-DROBOT_WALK_TURN_SETTLE_MS=300U')}
)
try {
    $env:PATH=(Split-Path $compiler)+[IO.Path]::PathSeparator+$env:PATH
    foreach($case in $cases){
        $binary=Join-Path $build ($case.Name+'.exe')
        & $compiler @base @($case.Defines) '-o' $binary
        if($LASTEXITCODE -ne 0){throw ('Compilation failed: '+$case.Name)}
        & $binary | Tee-Object -FilePath (Join-Path $build ($case.Name+'.log'))
        if($LASTEXITCODE -ne 0){throw ('Tests failed: '+$case.Name)}
        Write-Output ('CASE PASSED: '+$case.Name)
    }
    foreach($property in $state.protected_sha256.PSObject.Properties){
        $actual=(Get-FileHash -LiteralPath (Join-Path $root $property.Name) -Algorithm SHA256).Hash
        if($actual.ToLowerInvariant() -ne $property.Value){throw ('Protected file changed: '+$property.Name)}
    }
    Write-Output 'ALL 6 CONFIGURATIONS PASSED; MOTION AND VISION SOURCES UNCHANGED'
}finally{$env:PATH=$taskSavedPath}
