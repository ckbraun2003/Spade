<#
.SYNOPSIS
    Prints the path of a Python 3 that is NOT the Microsoft Store's, or prints nothing.

.DESCRIPTION
    scripts/build.ps1 passes the answer to a new tree's configure as -DPython3_EXECUTABLE (cmake/SpadePython.cmake says why: the Store's
    Python is an app-execution alias under C:\Program Files\WindowsApps, and starting it from a build tool sometimes fails with "Access
    is denied"). The generated files are byte-identical under any Python 3, so this choice changes no output.

    Order: python.org installs registered under PythonCore (HKCU, then HKLM), then MSYS2's native Python (C:\msys64\ucrt64\bin).
    A candidate counts only if its python.exe exists, is outside WindowsApps, and runs: a registry entry without an install path and a
    package manager's shim whose target is gone are skipped.
#>
[CmdletBinding()]
param()
$ErrorActionPreference = 'SilentlyContinue'

$candidates = New-Object System.Collections.Generic.List[string]
foreach ($hive in 'HKCU:', 'HKLM:', 'HKLM:\SOFTWARE\WOW6432Node') {
    $core = if ($hive -like '*WOW6432Node') { "$hive\Python\PythonCore" } else { "$hive\Software\Python\PythonCore" }
    foreach ($ver in (Get-ChildItem $core | Sort-Object { try { [version]($_.PSChildName -replace '-.*$', '') } catch { [version]'0.0' } } -Descending)) {
        $dir = (Get-ItemProperty -Path (Join-Path $ver.PSPath 'InstallPath')).'(default)'
        if (-not $dir) { $dir = (Get-ItemProperty -Path (Join-Path $ver.PSPath 'InstallPath')).ExecutablePath }
        if ($dir) { $candidates.Add($(if ($dir -like '*.exe') { $dir } else { Join-Path $dir 'python.exe' })) }
    }
}
$candidates.Add('C:\msys64\ucrt64\bin\python.exe')

foreach ($exe in $candidates) {
    if ($exe -like '*WindowsApps*') { continue }
    if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) { continue }
    $version = & $exe -c 'import sys; print(sys.version_info[0])' 2>$null
    if ($LASTEXITCODE -eq 0 -and "$version".Trim() -eq '3') { $exe.Replace('\', '/'); exit 0 }
}
exit 0
