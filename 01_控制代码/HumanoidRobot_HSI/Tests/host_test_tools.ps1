function Resolve-HostTestCompiler {
    param([string]$Compiler = 'gcc')

    $command = Get-Command -Name $Compiler -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($null -eq $command) {
        throw "找不到 GCC：$Compiler。请将 GCC 加入 PATH，或通过 -Compiler 指定 gcc.exe 的完整路径。"
    }
    return $command.Source
}

function Get-HostSourceSnapshot {
    param([string]$ProjectRoot)

    $hashes = [ordered]@{}
    $extensions = @('.c', '.h', '.s', '.ioc', '.py', '.uvprojx', '.uvoptx', '.sct')
    foreach ($file in Get-ChildItem -LiteralPath $ProjectRoot -Recurse -File |
        Where-Object { $_.Extension -in $extensions -and $_.FullName -notmatch '[\\/][^\\/]+_build[\\/]' }) {
        $relative = [IO.Path]::GetRelativePath($ProjectRoot, $file.FullName)
        $hashes[$relative] = (Get-FileHash -LiteralPath $file.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
    return [pscustomobject]@{ protected_sha256 = [pscustomobject]$hashes }
}
