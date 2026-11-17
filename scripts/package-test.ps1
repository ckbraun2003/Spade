<#
.SYNOPSIS
    Tests a Spade Editor setup.exe the way a tester meets it: install into a temporary per-user folder, run it, uninstall, check
    that nothing is left behind except %LOCALAPPDATA%\Spade\logs.

.DESCRIPTION
    Rerunnable; no administrator rights needed (the installer is per-user). Steps, each printed as PASS or FAIL:
      1  install silently into a fresh temp folder (/CURRENTUSER, /DIR=...): exit 0; the uninstaller is registered under HKCU, not HKLM
      2  the file list: the editor, assets, licence, notices, release docs, the guide; no tests, no flow_movie, no SDK files
      3  the shortcuts: Spade Editor, Spade Drone Box and Spade User Guide in the Start menu; Spade Editor on the desktop
      4  `spade_sandbox.exe --version` from the install folder prints the version the setup.exe's name carries
         (and `spade_geo_fetch.exe` starts there: --help exits 0, and --check on an empty cache is a miss, exit 3, with no network)
      5  the live smoke from the install folder (`--live-smoke --assets <install>\assets`): exit 0 and the journal ends END verdict=PASS
         (needs a display and the GPU; -SkipSmoke leaves it out)
      6  uninstall silently: the folder, the Start-menu folder, both shortcuts and the HKCU entry are gone, and %LOCALAPPDATA%\Spade holds
         nothing but logs
    Exit code 0 when every step passes, 1 otherwise; the last line says which failed.

.PARAMETER Setup
    The setup.exe. Default: the newest build-package\Spade-Editor-*-setup.exe (scripts\package.ps1 makes it).

.PARAMETER SmokeOut
    Where the smoke's journal and stills go. Default: a folder under the temp folder, removed with the run.

.PARAMETER SkipSmoke
    Leave out step 5 (no display, or a quick install check).

.PARAMETER Keep
    Do not uninstall at the end (for looking at an install by hand; step 6 is then not run and the exit code says so).

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\package-test.ps1
#>
[CmdletBinding()]
param(
    [string] $Setup,
    [string] $SmokeOut,
    [switch] $SkipSmoke,
    [switch] $Keep
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot

if (-not $Setup) {
    $found = Get-ChildItem -LiteralPath (Join-Path $repo 'build-package') -Filter 'Spade-Editor-*-setup.exe' -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($found) { $Setup = $found.FullName }
}
if (-not $Setup -or -not (Test-Path -LiteralPath $Setup)) {
    Write-Host 'package-test: no setup.exe (run scripts\package.ps1, or pass -Setup)'
    exit 2
}
$Setup = (Resolve-Path -LiteralPath $Setup).Path
$setupName = [IO.Path]::GetFileNameWithoutExtension($Setup)
if ($setupName -notmatch '^Spade-Editor-(?<version>\d+\.\d+\.\d+)-(?<tag>[0-9a-f]{7}(?:-dirty)?)-setup$') {
    Write-Host "package-test: '$setupName' is not Spade-Editor-<version>-<sha7>-setup"
    exit 2
}
$expectVersion = "$($Matches.version)+$($Matches.tag)"

$work = Join-Path $env:TEMP ("spade-install-test-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
$dir = Join-Path $work 'Spade Editor'
$logs = Join-Path $env:LOCALAPPDATA 'Spade'
if (-not $SmokeOut) { $SmokeOut = Join-Path $work 'smoke' }
New-Item -ItemType Directory -Force -Path $work | Out-Null

$failed = New-Object System.Collections.Generic.List[string]
function Check([string] $Name, [bool] $Ok, [string] $Detail = '') {
    if ($Ok) { Write-Host "package-test: PASS  $Name" }
    else { Write-Host "package-test: FAIL  $Name $Detail"; $failed.Add($Name) | Out-Null }
}
function Run([string] $Exe, [string[]] $ArgList, [string] $Cwd = $work) {
    # Output and exit code, a native command's stderr never stopping the script.
    $old = $ErrorActionPreference; $ErrorActionPreference = 'Continue'
    Push-Location $Cwd
    try { $out = & $Exe @ArgList 2>&1 | Out-String; $code = $LASTEXITCODE } finally { Pop-Location; $ErrorActionPreference = $old }
    [pscustomobject]@{ Output = $out.Trim(); Code = $code }
}
function UninstallKeys {
    Get-ChildItem 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall', 'HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall' -ErrorAction SilentlyContinue |
        ForEach-Object { Get-ItemProperty $_.PSPath } | Where-Object { $_.DisplayName -like 'Spade Editor*' } | ForEach-Object { $_.PSPath }
}
$startMenu = Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs'
$desktop = [Environment]::GetFolderPath('Desktop')

Write-Host "package-test: setup $Setup"
Write-Host "package-test: into  $dir"

# Whatever is already installed would make the checks meaningless.
$before = @(UninstallKeys)
if ($before) { Write-Host 'package-test: a Spade Editor is already installed for this user; uninstall it first'; Remove-Item -Recurse -Force $work; exit 2 }

# ---- 1 install ------------------------------------------------------------------------------------------------------------------------
$installLog = Join-Path $work 'install.log'
$p = Start-Process -FilePath $Setup -ArgumentList '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', '/CURRENTUSER', '/TASKS=desktopicon', "`"/DIR=$dir`"", "`"/LOG=$installLog`"" -Wait -PassThru
Check 'install exits 0' ($p.ExitCode -eq 0) "(exit $($p.ExitCode); log $installLog)"
$keys = @(UninstallKeys)
Check 'uninstaller registered under HKCU only (per-user, no administrator)' ($keys.Count -eq 1 -and $keys[0] -like '*HKEY_CURRENT_USER*') "($($keys -join '; '))"

# ---- 2 file list ----------------------------------------------------------------------------------------------------------------------
$must = 'spade_sandbox.exe', 'spade_geo_fetch.exe', 'LICENSE', 'THIRD_PARTY_NOTICES.md', 'VERSION', 'INSTALL.md', 'RELEASE_NOTES.md', 'SYSTEM_REQUIREMENTS.md',
        'assets\library', 'assets\scenes', 'assets\worlds', 'assets\fonts\Inter-Regular.ttf', 'docs\user-guide\index.html', 'unins000.exe'
$missing = @($must | Where-Object { -not (Test-Path -LiteralPath (Join-Path $dir $_)) })
Check 'installed files present' ($missing.Count -eq 0) "(missing: $($missing -join ', '))"
$pdf = @(Get-ChildItem -LiteralPath (Join-Path $dir 'docs\user-guide') -Filter '*.pdf' -ErrorAction SilentlyContinue)
Check 'the guide PDF is installed' ($pdf.Count -ge 1)
Check 'the MSVC runtime is app-local' ($null -ne (Get-ChildItem -LiteralPath $dir -Filter 'vcruntime140*.dll' -ErrorAction SilentlyContinue))
$stray = @(Get-ChildItem -LiteralPath $dir -Recurse -File -ErrorAction SilentlyContinue |
    Where-Object { $_.Extension -in '.lib', '.hpp', '.h', '.pdb', '.obj' -or $_.Name -in 'flow_movie.exe', 'spade_tests.exe', 'spade_bench.exe', 'spade_viewer.exe' })
Check 'nothing the editor does not need (no tests, flow_movie, SDK files)' ($stray.Count -eq 0) "($(($stray | ForEach-Object Name) -join ', '))"

# ---- 3 shortcuts ----------------------------------------------------------------------------------------------------------------------
$shell = New-Object -ComObject WScript.Shell
$group = Get-ChildItem -LiteralPath $startMenu -Directory -Filter 'Spade Editor*' -ErrorAction SilentlyContinue | Select-Object -First 1
$groupPath = if ($group) { $group.FullName } else { Join-Path $startMenu 'Spade Editor' }
$links = @{}
foreach ($n in 'Spade Editor', 'Spade Drone Box', 'Spade User Guide') {
    $lnk = Join-Path $groupPath "$n.lnk"
    if (Test-Path -LiteralPath $lnk) { $links[$n] = $shell.CreateShortcut($lnk) }
}
Check 'Start menu: Spade Editor, Spade Drone Box, Spade User Guide' ($links.Count -eq 3) "(found: $($links.Keys -join ', ') in $groupPath)"
if ($links.ContainsKey('Spade Drone Box')) {
    Check 'Spade Drone Box runs with --window' ($links['Spade Drone Box'].Arguments -eq '--window') "(arguments: '$($links['Spade Drone Box'].Arguments)')"
}
if ($links.ContainsKey('Spade Editor')) {
    Check 'Spade Editor starts the editor with no arguments, from the install folder' ($links['Spade Editor'].Arguments -eq '' -and $links['Spade Editor'].TargetPath -like "$dir\*") "(target $($links['Spade Editor'].TargetPath) $($links['Spade Editor'].Arguments))"
}
if ($links.ContainsKey('Spade User Guide')) {
    Check 'Spade User Guide opens the installed index.html' ($links['Spade User Guide'].TargetPath -eq (Join-Path $dir 'docs\user-guide\index.html')) "(target $($links['Spade User Guide'].TargetPath))"
}
$deskLink = Join-Path $desktop 'Spade Editor.lnk'
Check 'desktop: Spade Editor' (Test-Path -LiteralPath $deskLink)

# ---- 4 --version ----------------------------------------------------------------------------------------------------------------------
$exe = Join-Path $dir 'spade_sandbox.exe'
$v = Run $exe @('--version') $dir
Check "--version prints $expectVersion" ($v.Code -eq 0 -and $v.Output -like "*$expectVersion*") "(exit $($v.Code): '$($v.Output)')"

# ---- 4b the map fetch tool starts from the install folder, and with no network and an empty cache says miss (exit 3) -----------------------
$geoExe = Join-Path $dir 'spade_geo_fetch.exe'
$geoHelp = Run $geoExe @('--help') $dir
Check 'spade_geo_fetch --help exits 0 and prints its usage' ($geoHelp.Code -eq 0 -and $geoHelp.Output -like '*usage: spade_geo_fetch*') "(exit $($geoHelp.Code): '$($geoHelp.Output)')"
$geoCache = Join-Path $env:TEMP "spade-geo-check-$PID"
$geoCheck = Run $geoExe @('--lat', '40.76', '--lon', '-113.90', '--check', '--cache', $geoCache) $dir
Check 'spade_geo_fetch --check on an empty cache is a miss (exit 3), without the network' ($geoCheck.Code -eq 3 -and $geoCheck.Output -like '*miss*') "(exit $($geoCheck.Code): '$($geoCheck.Output)')"
Remove-Item -LiteralPath $geoCache -Recurse -Force -ErrorAction SilentlyContinue

# ---- 5 live smoke from the install folder ---------------------------------------------------------------------------------------------
if ($SkipSmoke) {
    Write-Host 'package-test: SKIP  live smoke (-SkipSmoke)'
} else {
    $s = Run $exe @('--live-smoke', '--out', $SmokeOut, '--label', "install-test $expectVersion", '--assets', (Join-Path $dir 'assets')) $dir
    $journal = Join-Path $SmokeOut 'journal.txt'
    $last = if (Test-Path -LiteralPath $journal) { (Get-Content -LiteralPath $journal | Where-Object { $_ } | Select-Object -Last 1) } else { '(no journal)' }
    Check 'live smoke from the install folder: exit 0, END verdict=PASS' ($s.Code -eq 0 -and $last -eq 'END verdict=PASS') "(exit $($s.Code); journal ends '$last'; $journal)"
}

# ---- 6 uninstall ----------------------------------------------------------------------------------------------------------------------
if ($Keep) {
    Write-Host "package-test: KEPT  the install at $dir (-Keep); uninstall with: `"$dir\unins000.exe`""
    Write-Host 'package-test: step 6 (uninstall) not run'
    exit 3
}
$unins = Join-Path $dir 'unins000.exe'
if (Test-Path -LiteralPath $unins) {
    Start-Process -FilePath $unins -ArgumentList '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART' -Wait | Out-Null
    # Inno's uninstaller re-launches a copy of itself from the temp folder and returns; wait for the folder to go.
    $deadline = (Get-Date).AddSeconds(90)
    while ((Test-Path -LiteralPath $unins) -and (Get-Date) -lt $deadline) { Start-Sleep -Milliseconds 500 }
    Start-Sleep -Seconds 2
}
$left = @()
if (Test-Path -LiteralPath $dir) { $left = @(Get-ChildItem -LiteralPath $dir -Recurse -Force -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName.Substring($dir.Length) }) }
Check 'uninstall: the install folder is gone' (-not (Test-Path -LiteralPath $dir)) "(left: $($left -join ', '))"
Check 'uninstall: the Add/Remove Programs entry is gone' (@(UninstallKeys).Count -eq 0)
Check 'uninstall: the Start-menu folder is gone' (-not (Test-Path -LiteralPath $groupPath))
Check 'uninstall: the desktop shortcut is gone' (-not (Test-Path -LiteralPath $deskLink))
$extra = @()
if (Test-Path -LiteralPath $logs) { $extra = @(Get-ChildItem -LiteralPath $logs -Force | Where-Object { $_.Name -ne 'logs' } | ForEach-Object Name) }
Check 'nothing left under %LOCALAPPDATA%\Spade except logs' ($extra.Count -eq 0) "(left: $($extra -join ', '))"
Check 'nothing left under %APPDATA%\Spade' (-not (Test-Path -LiteralPath (Join-Path $env:APPDATA 'Spade')))

Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue
if ($failed.Count -eq 0) {
    Write-Host "package-test: ALL PASS ($expectVersion)"
    exit 0
}
Write-Host "package-test: FAILED: $($failed -join '; ')"
exit 1
