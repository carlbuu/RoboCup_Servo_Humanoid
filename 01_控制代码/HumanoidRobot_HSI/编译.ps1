param(
    [string]$KeilExe = ''
)

$ErrorActionPreference = 'Stop'
if ([string]::IsNullOrWhiteSpace($KeilExe)) {
    if (-not [string]::IsNullOrWhiteSpace($env:KEIL_UV4)) {
        $KeilExe = $env:KEIL_UV4
    } else {
        $command = Get-Command 'UV4.exe' -CommandType Application -ErrorAction SilentlyContinue
        if ($null -ne $command) { $KeilExe = $command.Source }
    }
}
if ([string]::IsNullOrWhiteSpace($KeilExe)) {
    throw 'Specify -KeilExe with the UV4.exe path, set KEIL_UV4, or add UV4.exe to PATH.'
}
if (-not (Test-Path -LiteralPath $KeilExe)) {
    throw "找不到 Keil：$KeilExe"
}

# 使用临时路径进行批处理编译，生成物回写到本调试工程。
$buildRoot = Join-Path ([IO.Path]::GetTempPath()) ('humanoid_hsi_' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $buildRoot | Out-Null
foreach ($name in @('Core', 'Drivers', 'Middlewares', 'User')) {
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot $name) -Destination $buildRoot -Recurse
}
$mdk = Join-Path $buildRoot 'MDK-ARM'
New-Item -ItemType Directory -Path $mdk | Out-Null
foreach ($name in @('HumanoidRobot_HSI.uvprojx', 'HumanoidRobot_HSI.uvoptx', 'startup_stm32f407xx.s', 'RTE', 'DebugConfig')) {
    $item = Join-Path $PSScriptRoot ('MDK-ARM\' + $name)
    if (Test-Path -LiteralPath $item) {
        Copy-Item -LiteralPath $item -Destination $mdk -Recurse
    }
}

$project = Join-Path $mdk 'HumanoidRobot_HSI.uvprojx'
$log = Join-Path $mdk 'build_hsi_migration.log'
$process = Start-Process -FilePath $KeilExe -WindowStyle Hidden -PassThru -ArgumentList @(
    '-r', ('"' + $project + '"'), '-o', ('"' + $log + '"')
)
$process.WaitForExit()
if (Test-Path -LiteralPath $log) {
    Get-Content -LiteralPath $log
    Copy-Item -LiteralPath $log -Destination (Join-Path $PSScriptRoot 'MDK-ARM\build_hsi_migration.log') -Force
}
if ($process.ExitCode -ne 0) {
    throw "Keil 编译失败，ExitCode=$($process.ExitCode)。临时目录：$buildRoot"
}

$output = Join-Path $PSScriptRoot 'MDK-ARM\HumanoidRobot_HSI'
New-Item -ItemType Directory -Path $output -Force | Out-Null
foreach ($ext in @('axf', 'hex', 'map', 'htm')) {
    $file = Join-Path $mdk ('HumanoidRobot_HSI\HumanoidRobot_HSI.' + $ext)
    if (Test-Path -LiteralPath $file) {
        Copy-Item -LiteralPath $file -Destination $output -Force
    }
}
Write-Output "生成物已保存：$output"
Write-Output "编译临时目录：$buildRoot"
