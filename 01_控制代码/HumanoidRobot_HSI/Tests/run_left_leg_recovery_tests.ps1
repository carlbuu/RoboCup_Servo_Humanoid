param([string]$Compiler = 'gcc', [string]$BaselineVisionSource = '')
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'host_test_tools.ps1')
$Compiler = Resolve-HostTestCompiler -Compiler $Compiler
if ([string]::IsNullOrWhiteSpace($BaselineVisionSource) -or -not (Test-Path -LiteralPath $BaselineVisionSource -PathType Leaf)) {
    throw 'This suite needs matching reference scheduler source. Specify -BaselineVisionSource; the reference file is not distributed.'
}
$root = Split-Path $PSScriptRoot
$state = Get-HostSourceSnapshot -ProjectRoot $root
$build = Join-Path $root 'Tests/left_leg_recovery_build'
$null = New-Item -ItemType Directory -Path $build -Force
$taskSavedPath = $env:PATH
$base = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic',
    '-I', (Join-Path $root 'Tests/host_stubs'), '-I', (Join-Path $root 'Tests'),
    '-I', (Join-Path $root 'Drivers/camera'), '-I', (Join-Path $root 'User/motion'),
    '-I', (Join-Path $root 'User/servo'),
    (Join-Path $root 'Tests/test_left_leg_recovery.c'),
    (Join-Path $root 'Drivers/camera/camera.c'),
    (Join-Path $root 'User/motion/robot_control.c'),
    (Join-Path $root 'User/motion/motion_player.c'))
function Run-Case([string]$name, [string[]]$defines, [string]$vision, [string]$groups, [bool]$expectedFailure = $false) {
    $binary = Join-Path $build ($name + '.exe')
    & $compiler @base @defines $vision $groups '-o' $binary
    if ($LASTEXITCODE -ne 0) { throw ('Compilation failed: ' + $name) }
    $log = Join-Path $build ($name + '.log')
    & $binary 2>&1 | Tee-Object -FilePath $log
    $caseExit = $LASTEXITCODE
    if ($expectedFailure) {
        if ($caseExit -eq 0 -or !(Select-String -LiteralPath $log -SimpleMatch 'ROBOT_WALK_SETTLING' -Quiet)) {
            throw 'Pre-change source must fail at the duplicate recovery assertion'
        }
        Write-Output 'PASS regression test detects the original duplicate recovery'
    } elseif ($caseExit -ne 0) { throw ('Host test failed: ' + $name) }
    Write-Output ('CASE PASSED: ' + $name)
}
try {
    $env:PATH = (Split-Path $compiler) +[IO.Path]::PathSeparator+ $env:PATH
    $vision = Join-Path $root 'User/motion/robot_walk_vision.c'
    $groups = Join-Path $root 'User/motion/motion_groups.c'
    Run-Case 'before_change' @('-DROBOT_RUN_MODE=3U') $BaselineVisionSource $groups $true
    foreach($mode in @(2,3)) {
        foreach($protocol in @(0,1)) {
            Run-Case ('mode' + $mode + '_protocol' + $protocol) @(('-DROBOT_RUN_MODE=' + $mode + 'U'), ('-DCAMERA_PROTOCOL_BINARY=' + $protocol + 'U')) $vision $groups
        }
    }
    Run-Case 'line_mode_unchanged' @('-DROBOT_RUN_MODE=4U') $vision $groups
    Run-Case 'action_mode_unchanged' @('-DROBOT_RUN_MODE=1U', '-DROBOT_DEBUG_MOTION=motion_qr_right_leg') $vision $groups
    $s = [IO.File]::ReadAllText($groups)
    $pattern = '(?s)(static const MotionFrame_t s_qrLeftLeg\[\]\s*=\s*\{)(.*?)(\r?\n\};)'
    $match = [regex]::Match($s, $pattern)
    if (!$match.Success) { throw 'Left-leg fixture not found' }
    $oldBody = $match.Groups[2].Value
    $newBody = [regex]::Replace($oldBody, '(\{\s*1000\s*,\s*\{)\s*0', '$1 1')
    if ($oldBody -eq $newBody) { throw 'Nonzero final target fixture failed' }
    $s = $s.Substring(0, $match.Groups[2].Index) + $newBody + $s.Substring($match.Groups[2].Index + $oldBody.Length)
    $fixture = Join-Path $build 'motion_groups_nonzero_fixture.c'
    [IO.File]::WriteAllText($fixture, $s, [Text.UTF8Encoding]::new($false))
    Run-Case 'nonzero_left_end_fallback' @('-DROBOT_RUN_MODE=3U', '-DTEST_NONZERO_LEFT_END=1') $vision $fixture
    foreach($property in $state.protected_sha256.PSObject.Properties) {
        $actual = (Get-FileHash -LiteralPath (Join-Path $root $property.Name) -Algorithm SHA256).Hash
        if ($actual.ToLowerInvariant() -ne $property.Value) { throw ('Protected source changed: ' + $property.Name) }
    }
    Write-Output ('PASS ' + @($state.protected_sha256.PSObject.Properties).Count + ' protected source hashes unchanged')
    Write-Output 'ALL 7 CURRENT CONFIGURATIONS PASSED; PRE-CHANGE REGRESSION CONFIRMED'
} finally { $env:PATH = $taskSavedPath }
