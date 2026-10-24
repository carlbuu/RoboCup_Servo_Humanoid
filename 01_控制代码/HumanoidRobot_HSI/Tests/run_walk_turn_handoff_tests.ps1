param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$root=Split-Path $PSScriptRoot
$build=Join-Path $PSScriptRoot 'walk_turn_handoff_build'
$null=New-Item -ItemType Directory -Path $build -Force
$savedPath=$env:PATH
try {
    $env:PATH=(Split-Path $Compiler)+[IO.Path]::PathSeparator+$env:PATH
    foreach($mode in @(3,4)) {
        foreach($protocol in @(0,1)) {
            foreach($each in @(1,0)) {
                $name="mode${mode}_protocol${protocol}_each${each}"
                $exe=Join-Path $build ($name+'.exe')
                $argsList=@('-std=c99','-Wall','-Wextra','-Werror','-pedantic',
                    "-DROBOT_RUN_MODE=${mode}U","-DCAMERA_PROTOCOL_BINARY=${protocol}U",
                    "-DROBOT_WALK_DECIDE_EACH_CYCLE=${each}U",
                    '-I',(Join-Path $PSScriptRoot 'host_stubs'),'-I',$PSScriptRoot,
                    '-I',(Join-Path $root 'Drivers/camera'),'-I',(Join-Path $root 'User/motion'),
                    '-I',(Join-Path $root 'User/servo'),
                    (Join-Path $PSScriptRoot 'test_walk_turn_handoff.c'),
                    (Join-Path $root 'Drivers/camera/camera.c'),
                    (Join-Path $root 'User/motion/robot_control.c'),
                    (Join-Path $root 'User/motion/robot_walk_vision.c'),
                    (Join-Path $root 'User/motion/motion_player.c'),
                    (Join-Path $root 'User/motion/motion_groups.c'),'-o',$exe)
                & $Compiler @argsList
                if($LASTEXITCODE -ne 0){throw "Compile failed: $name"}
                & $exe | Set-Content -LiteralPath (Join-Path $build ($name+'.log')) -Encoding utf8
                if($LASTEXITCODE -ne 0){throw "Tests failed: $name"}
                Write-Output "PASS $name"
            }
        }
    }
} finally {$env:PATH=$savedPath}
