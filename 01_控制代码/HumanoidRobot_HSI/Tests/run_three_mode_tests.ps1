param(
    [string]$Compiler = 'gcc',
    [string]$VisionSource = ''
)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
$projectRoot = Split-Path $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($VisionSource)) {
    $VisionSource = Join-Path (Split-Path (Split-Path $projectRoot)) '02_视觉代码/geometry_vision19.py'
}
$build = Join-Path $PSScriptRoot 'three_modes_build'
New-Item -ItemType Directory -Force -Path $build | Out-Null
$pythonFiles = @(Get-ChildItem -LiteralPath $projectRoot -Recurse -File -Filter '*.py')
if (Test-Path -LiteralPath $VisionSource) {
    $pythonFiles += Get-Item -LiteralPath $VisionSource
}
$pythonHashes = @{}
foreach ($file in $pythonFiles) {
    $pythonHashes[$file.FullName] = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash
}
$cases = @(
    @{ Name = 'line_ascii_two_steps'; Defines = @('-DROBOT_RUN_MODE=4U') },
    @{ Name = 'line_binary_two_steps'; Defines = @('-DROBOT_RUN_MODE=4U', '-DCAMERA_PROTOCOL_BINARY=1U') },
    @{ Name = 'line_ascii_four_steps'; Defines = @('-DROBOT_RUN_MODE=4U', '-DROBOT_STM_WALK_BATCH_STEPS=4U') },
    @{ Name = 'line_autostart_override'; Defines = @('-DROBOT_RUN_MODE=4U', '-DROBOT_STM_AUTOSTART=1U') },
    @{ Name = 'action_right_leg'; Defines = @('-DROBOT_RUN_MODE=1U', '-DROBOT_DEBUG_MOTION=motion_qr_right_leg') },
    @{ Name = 'action_stand'; Defines = @('-DROBOT_RUN_MODE=1U', '-DROBOT_DEBUG_MOTION=motion_stand') },
    @{ Name = 'action_walk_loop'; Defines = @('-DROBOT_RUN_MODE=1U', '-DROBOT_DEBUG_MOTION=motion_walk_forward') },
    @{ Name = 'ascii_two_steps'; Defines = @('-DROBOT_RUN_MODE=3U', '-DROBOT_STM_AUTOSTART=1U') },
    @{ Name = 'ascii_four_steps'; Defines = @('-DROBOT_RUN_MODE=3U', '-DROBOT_STM_AUTOSTART=1U', '-DROBOT_STM_WALK_BATCH_STEPS=4U') },
    @{ Name = 'binary_two_steps'; Defines = @('-DROBOT_RUN_MODE=3U', '-DROBOT_STM_AUTOSTART=1U', '-DCAMERA_PROTOCOL_BINARY=1U') },
    @{ Name = 'shape_only_ascii'; Defines = @('-DROBOT_RUN_MODE=2U') },
    @{ Name = 'shape_only_binary'; Defines = @('-DROBOT_RUN_MODE=2U', '-DCAMERA_PROTOCOL_BINARY=1U') },
    @{ Name = 'shape_only_autostart_override'; Defines = @('-DROBOT_RUN_MODE=2U', '-DROBOT_STM_AUTOSTART=1U') }
)
$taskSavedPath = $env:PATH
try {
    $env:PATH = (Split-Path $Compiler) +[IO.Path]::PathSeparator+ $env:PATH
    foreach ($case in $cases) {
        $binary = Join-Path $build ($case.Name + '.exe')
        $compilerArgs = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
            '-I', (Join-Path $PSScriptRoot 'host_stubs'),
            '-I', (Join-Path $projectRoot 'Drivers/camera'),
            '-I', (Join-Path $projectRoot 'User/motion'),
            '-I', (Join-Path $projectRoot 'User/servo'),
            (Join-Path $PSScriptRoot 'test_three_modes.c'),
            (Join-Path $projectRoot 'Drivers/camera/camera.c'),
            (Join-Path $projectRoot 'User/motion/robot_control.c'),
            (Join-Path $projectRoot 'User/motion/robot_walk_vision.c'),
            (Join-Path $projectRoot 'User/motion/motion_player.c'),
            (Join-Path $projectRoot 'User/motion/motion_groups.c'),
            '-o', $binary)
        & $Compiler @compilerArgs @($case.Defines)
        if ($LASTEXITCODE -ne 0) { throw ('Host compilation failed: ' + $case.Name) }
        Write-Output ('CASE ' + $case.Name)
        & $binary | Tee-Object -FilePath (Join-Path $build ($case.Name + '.log'))
        if ($LASTEXITCODE -ne 0) { throw ('Host test failed: ' + $case.Name) }
    }
    foreach ($path in $pythonHashes.Keys) {
        $actual = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash
        if ($actual -ne $pythonHashes[$path]) { throw ('Python source changed: ' + $path) }
    }
    Write-Output ('PASS ' + $pythonHashes.Count + ' original Python source hashes unchanged')
} finally {
    $env:PATH = $taskSavedPath
}
