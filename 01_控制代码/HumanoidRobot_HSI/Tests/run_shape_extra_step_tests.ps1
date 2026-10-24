param([string]$Compiler = 'gcc')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$root = Split-Path $PSScriptRoot
$state = Get-HostSourceSnapshot -ProjectRoot $root
$build=Join-Path $root 'Tests/shape_extra_step_build'
$null=New-Item -ItemType Directory -Path $build -Force
$base=@('-std=c99','-Wall','-Wextra','-Werror','-pedantic',
    '-I',(Join-Path $root 'Tests/host_stubs'),'-I',(Join-Path $root 'Tests'),
    '-I',(Join-Path $root 'Drivers/camera'),'-I',(Join-Path $root 'User/motion'),'-I',(Join-Path $root 'User/servo'),
    (Join-Path $root 'Tests/test_shape_extra_step.c'),(Join-Path $root 'Drivers/camera/camera.c'),
    (Join-Path $root 'User/motion/robot_control.c'),(Join-Path $root 'User/motion/robot_walk_vision.c'),
    (Join-Path $root 'User/motion/motion_player.c'),(Join-Path $root 'User/motion/motion_groups.c'))
$cases=@(
    @{Name='full_ascii';Defs=@('-DROBOT_RUN_MODE=3U')},
    @{Name='full_binary';Defs=@('-DROBOT_RUN_MODE=3U','-DCAMERA_PROTOCOL_BINARY=1U')},
    @{Name='full_two_step_batch';Defs=@('-DROBOT_RUN_MODE=3U','-DROBOT_STM_WALK_BATCH_STEPS=2U')},
    @{Name='feature_disabled';Defs=@('-DROBOT_RUN_MODE=3U','-DROBOT_SHAPE_STEP_BEFORE_ACTION=0U')},
    @{Name='image_ascii';Defs=@('-DROBOT_RUN_MODE=2U')},
    @{Name='image_binary';Defs=@('-DROBOT_RUN_MODE=2U','-DCAMERA_PROTOCOL_BINARY=1U')},
    @{Name='line_mode';Defs=@('-DROBOT_RUN_MODE=4U')},
    @{Name='action_mode';Defs=@('-DROBOT_RUN_MODE=1U','-DROBOT_DEBUG_MOTION=motion_qr_right_leg')}
)
$taskSavedPath=$env:PATH
try{
    $env:PATH=(Split-Path $compiler)+[IO.Path]::PathSeparator+$env:PATH
    foreach($case in $cases){
        $binary=Join-Path $build ($case.Name+'.exe')
        & $compiler @base @($case.Defs) '-o' $binary
        if($LASTEXITCODE -ne 0){throw ('Compile failed: '+$case.Name)}
        & $binary | Tee-Object -FilePath (Join-Path $build ($case.Name+'.log'))
        if($LASTEXITCODE -ne 0){throw ('Test failed: '+$case.Name)}
        Write-Output ('CASE PASSED: '+$case.Name)
    }
    foreach($property in $state.protected_sha256.PSObject.Properties){
        $actual=(Get-FileHash -LiteralPath (Join-Path $root $property.Name) -Algorithm SHA256).Hash
        if($actual.ToLowerInvariant() -ne $property.Value){throw ('Protected source changed: '+$property.Name)}
    }
    Write-Output 'ALL 8 CONFIGURATIONS PASSED; MOTION, SERVO, PLAYER AND CAMERA SOURCES UNCHANGED'
}finally{$env:PATH=$taskSavedPath}
