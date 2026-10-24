param([string]$Compiler='gcc',
    [ValidateSet(0,1)][int]$ShapeExtra=0,
    [ValidateSet(0,1)][int]$EachCycle=1)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$projectRoot=Split-Path $PSScriptRoot
$build=Join-Path $PSScriptRoot 'walk_late_vision_build'
$null=New-Item -ItemType Directory -Path $build -Force
$savedPath=$env:PATH
try {
    $env:PATH=(Split-Path $Compiler)+[IO.Path]::PathSeparator+$env:PATH
    foreach($mode in @(3,4)) {
        foreach($protocol in @(0,1)) {
            $suffix=if($ShapeExtra -eq 0){''}else{'_extra1'}
            if($EachCycle -eq 0){$suffix+='_lastcycle'}
            $exe=Join-Path $build "mode${mode}_protocol${protocol}${suffix}.exe"
            $compilerArgs=@('-std=c99','-Wall','-Wextra','-Werror','-pedantic',
                "-DROBOT_RUN_MODE=${mode}U","-DCAMERA_PROTOCOL_BINARY=${protocol}U",
                "-DROBOT_SHAPE_STEP_BEFORE_ACTION=${ShapeExtra}U",
                "-DROBOT_WALK_DECIDE_EACH_CYCLE=${EachCycle}U",
                '-DROBOT_WALK_TURN_HANDOFF_ENABLE=0U', # 此套保留原收脚纠偏行为；新入口另测。
                '-I',(Join-Path $PSScriptRoot 'host_stubs'),'-I',$PSScriptRoot,
                '-I',(Join-Path $projectRoot 'Drivers/camera'),'-I',(Join-Path $projectRoot 'User/motion'),
                '-I',(Join-Path $projectRoot 'User/servo'),
                (Join-Path $PSScriptRoot 'test_walk_late_vision.c'),
                (Join-Path $projectRoot 'Drivers/camera/camera.c'),
                (Join-Path $projectRoot 'User/motion/robot_control.c'),
                (Join-Path $projectRoot 'User/motion/robot_walk_vision.c'),
                (Join-Path $projectRoot 'User/motion/motion_player.c'),
                (Join-Path $projectRoot 'User/motion/motion_groups.c'),'-o',$exe)
            & $Compiler @compilerArgs
            if($LASTEXITCODE -ne 0) { throw "Compile failed: mode=$mode protocol=$protocol" }
            & $exe | Set-Content -LiteralPath (Join-Path $build "mode${mode}_protocol${protocol}${suffix}.log") -Encoding utf8
            if($LASTEXITCODE -ne 0) { throw "Tests failed: mode=$mode protocol=$protocol" }
            Write-Output "PASS mode=$mode protocol=$protocol shape_extra=$ShapeExtra each_cycle=$EachCycle"
        }
    }
} finally { $env:PATH=$savedPath }
