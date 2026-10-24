param([string]$Compiler='gcc')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$root=Split-Path $PSScriptRoot
$build=Join-Path $PSScriptRoot 'motion_timing_audit_build'
$null=New-Item -ItemType Directory -Path $build -Force
$argsList=@('-std=c99','-Wall','-Wextra','-Werror','-pedantic','-DROBOT_RUN_MODE=3U',
 '-I',(Join-Path $PSScriptRoot 'host_stubs'),'-I',$PSScriptRoot,
 '-I',(Join-Path $root 'Drivers/camera'),'-I',(Join-Path $root 'User/motion'),'-I',(Join-Path $root 'User/servo'),
 (Join-Path $PSScriptRoot 'test_motion_timing_audit.c'),(Join-Path $root 'Drivers/camera/camera.c'),
 (Join-Path $root 'User/motion/robot_control.c'),(Join-Path $root 'User/motion/robot_walk_vision.c'),
 (Join-Path $root 'User/motion/motion_player.c'),(Join-Path $root 'User/motion/motion_groups.c'),
 '-o',(Join-Path $build 'audit.exe'))
$taskSavedPath=$env:PATH
try {
 $env:PATH=(Split-Path $Compiler)+[IO.Path]::PathSeparator+$env:PATH
 & $Compiler @argsList
 if($LASTEXITCODE -ne 0){throw 'Audit compile failed'}
 & (Join-Path $build 'audit.exe') | Tee-Object -FilePath (Join-Path $build 'audit.log')
 if($LASTEXITCODE -ne 0){throw 'Audit failed'}
}finally{$env:PATH=$taskSavedPath}
