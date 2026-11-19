<#
.SYNOPSIS
    Tests a Spade Editor setup.exe the way a tester meets it: install into a temporary per-user folder, run it, uninstall, check
    that nothing is left behind except %LOCALAPPDATA%\Spade\logs. Leaves a record of the test beside the setup.exe.

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

    The record (every run that gets as far as installing): `<setup name>.test.md` beside the setup.exe holds the UTC date and time, the
    commits, the setup's size and sha256, the machine (Windows build, CPU, memory, GPU and driver), every check with PASS or FAIL and its
    detail, the smoke journal's summary and a still's path, the job ticket and the total time. `<setup name>.test.log` is the full log.
    When the smoke ran, `<setup name>.smoke\` keeps its journal, the tour video and the numbered chapter stills (a pass or a fail alike).
    The record's shape is checked by tests\package_test_record_test.ps1.

.PARAMETER Setup
    The setup.exe. Default: the newest build-package\Spade-Editor-*-setup.exe (scripts\package.ps1 makes it).

.PARAMETER SmokeOut
    Where the smoke's journal and stills go while it runs. Default: a folder under the temp folder, removed with the run (the journal, the
    video and the numbered stills are first copied into the kept folder).

.PARAMETER SkipSmoke
    Leave out step 5 (no display, or a quick install check). The record then says the installer is not smoked.

.PARAMETER Keep
    Do not uninstall at the end (for looking at an install by hand; step 6 is then not run and the exit code says so).

.PARAMETER Ticket
    The Vantage job ticket this run is queued under, for the record. Default: the VANTAGE_TICKET variable, else "(none: run by hand)".

.PARAMETER RecordDir
    Where the record, the log and the kept smoke files go. Default: the setup.exe's own folder.

.PARAMETER ScriptRef
    What to print as the commit of this script, when it runs from a copy that is not a git checkout. Default: this checkout's HEAD.

.PARAMETER LedgerDir
    A folder that holds installer-ledger.md. Given, the run appends its row there and writes a copy of the record into <LedgerDir>\installer-records\,
    with the profile, local-app-data and roaming-app-data folders written as %USERPROFILE%, %LOCALAPPDATA% and %APPDATA% (the repo is public).
    The ledger is the part that goes into git (docs\design\test-docs).

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\package-test.ps1
#>
[CmdletBinding()]
param(
    [string] $Setup,
    [string] $SmokeOut,
    [switch] $SkipSmoke,
    [switch] $Keep,
    [string] $Ticket,
    [string] $RecordDir,
    [string] $LedgerDir,
    [string] $ScriptRef
)
$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'package-test-record.ps1')
$clock = [Diagnostics.Stopwatch]::StartNew()
$startedUtc = [DateTime]::UtcNow
$log = New-Object System.Collections.Generic.List[string]
function Say([string] $Message) { Write-Host $Message; $log.Add($Message) | Out-Null }

if (-not $Setup) {
    $found = Get-ChildItem -LiteralPath (Join-Path $repo 'build-package') -Filter 'Spade-Editor-*-setup.exe' -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
    if ($found) { $Setup = $found.FullName }
}
if (-not $Setup -or -not (Test-Path -LiteralPath $Setup)) {
    Say 'package-test: no setup.exe (run scripts\package.ps1, or pass -Setup)'
    exit 2
}
$Setup = (Resolve-Path -LiteralPath $Setup).Path
$setupName = [IO.Path]::GetFileNameWithoutExtension($Setup)
if ($setupName -notmatch '^Spade-Editor-(?<version>\d+\.\d+\.\d+)-(?<tag>[0-9a-f]{7}(?:-dirty)?)-setup$') {
    Say "package-test: '$setupName' is not Spade-Editor-<version>-<sha7>-setup"
    exit 2
}
$expectVersion = "$($Matches.version)+$($Matches.tag)"
$setupCommit = $Matches.tag
if (-not $RecordDir) { $RecordDir = Split-Path -Parent $Setup }
if (-not $Ticket) { $Ticket = $env:VANTAGE_TICKET }
if (-not $Ticket) { $Ticket = '(none: run by hand)' }

$work = Join-Path $env:TEMP ("spade-install-test-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
$dir = Join-Path $work 'Spade Editor'
$logs = Join-Path $env:LOCALAPPDATA 'Spade'
if (-not $SmokeOut) { $SmokeOut = Join-Path $work 'smoke' }
New-Item -ItemType Directory -Force -Path $work | Out-Null

$failed = New-Object System.Collections.Generic.List[string]
$checks = New-Object System.Collections.Generic.List[object]
function Check([string] $Name, [bool] $Ok, [string] $Detail = '') {
    $checks.Add([pscustomobject]@{ Name = $Name; Ok = $Ok; Detail = $Detail }) | Out-Null
    if ($Ok) { Say "package-test: PASS  $Name" }
    else { Say "package-test: FAIL  $Name $Detail"; $failed.Add($Name) | Out-Null }
}
function Run([string] $Exe, [string[]] $ArgList, [string] $Cwd = $work) {
    # Output and exit code, a native command's stderr never stopping the script. A missing program is a failed run, not a crash.
    if (-not (Test-Path -LiteralPath $Exe)) { return [pscustomobject]@{ Output = "not found: $Exe"; Code = -1 } }
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

# ---- the record's facts that exist before the install -----------------------------------------------------------------------------------
function MachineFacts {
    $m = @{ Windows = '(unknown)'; Cpu = '(unknown)'; Memory = '(unknown)'; Gpus = @() }
    try {
        $os = Get-CimInstance Win32_OperatingSystem
        $m.Windows = "$($os.Caption) $($os.Version) (build $($os.BuildNumber))"
        $m.Memory = ('{0:N1} GB' -f ($os.TotalVisibleMemorySize / 1MB))
        $m.Cpu = (@(Get-CimInstance Win32_Processor | ForEach-Object { $_.Name.Trim() }) -join '; ')
        $m.Gpus = @(Get-CimInstance Win32_VideoController | ForEach-Object { "$($_.Name), Windows driver $($_.DriverVersion)" })
    } catch { $m.Windows = "$($m.Windows) (CIM failed: $($_.Exception.Message))" }
    # The vendor's own driver number reads better than Windows' four-part one.
    $smi = Get-Command nvidia-smi -ErrorAction SilentlyContinue
    if ($smi) {
        try {
            $line = (& $smi.Source --query-gpu=name,driver_version --format=csv,noheader 2>$null | Select-Object -First 1)
            if ($line) { $m.Gpus = @("$line (nvidia-smi: name, driver)") + @($m.Gpus) }
        } catch { }
    }
    $m
}
$machine = MachineFacts
$scriptCommit = if ($ScriptRef) { $ScriptRef } else { 'unknown (not a git checkout)' }
$git = Get-Command git -ErrorAction SilentlyContinue
if (-not $ScriptRef -and $git -and (Test-Path -LiteralPath (Join-Path $repo '.git'))) {
    $h = (& $git.Source -C $repo rev-parse --short=8 HEAD 2>$null | Select-Object -First 1)
    if ($h) { $scriptCommit = $h }
}
$sha256 = (Get-FileHash -LiteralPath $Setup -Algorithm SHA256).Hash.ToLowerInvariant()
$setupBytes = (Get-Item -LiteralPath $Setup).Length
$versionPrinted = '(not run)'
$smokeInfo = $null
$smokeRan = $false
$recordBase = Join-Path $RecordDir $setupName

function Finish([int] $Code, [bool] $KeptInstall = $false) {
    # Writes the record and the log beside the setup, then the ledger row. Called on every way out after the install starts.
    $R = @{
        StartedUtc = $startedUtc; SetupPath = $Setup; SetupName = $setupName; SetupBytes = $setupBytes; Sha256 = $sha256
        SetupCommit = $setupCommit; ScriptCommit = $scriptCommit; ExpectVersion = $expectVersion; VersionPrinted = $versionPrinted
        Ticket = $Ticket; ElapsedSeconds = $clock.Elapsed.TotalSeconds; Machine = $machine; Checks = $checks.ToArray()
        SmokeRun = $smokeRan; Smoke = $smokeInfo; Kept = $KeptInstall; LogPath = "$recordBase.test.log"; Exit = $Code
    }
    try {
        New-Item -ItemType Directory -Force -Path $RecordDir | Out-Null
        $utf8 = New-Object System.Text.UTF8Encoding($false)
        [IO.File]::WriteAllText("$recordBase.test.md", ((New-InstallTestRecord $R) + "`n"), $utf8)
        [IO.File]::WriteAllText("$recordBase.test.log", (($log -join "`n") + "`n"), $utf8)
        Write-Host "package-test: record $recordBase.test.md"
        Write-Host "package-test: log    $recordBase.test.log"
        if ($LedgerDir) {
            $recDir = Join-Path $LedgerDir 'installer-records'
            New-Item -ItemType Directory -Force -Path $recDir | Out-Null
            # The ledger and its record copies go into the public repo: no home folder in them (the record beside the setup keeps the real paths).
            [IO.File]::WriteAllText((Join-Path $recDir "$setupName.test.md"), (ConvertTo-PublicRecord ((New-InstallTestRecord $R) + "`n")), $utf8)
            $ledger = Join-Path $LedgerDir 'installer-ledger.md'
            if (-not (Test-Path -LiteralPath $ledger)) { [IO.File]::WriteAllText($ledger, ($InstallLedgerHeader.Replace("`r", '') + "`n"), $utf8) }
            [IO.File]::AppendAllText($ledger, ((ConvertTo-PublicRecord (New-InstallLedgerRow $R "installer-records/$setupName.test.md")) + "`n"), $utf8)
            Write-Host "package-test: ledger row added to $ledger"
        }
    } catch {
        Write-Host "package-test: the record could not be written: $($_.Exception.Message)"
    }
}
trap {
    Say "package-test: FAIL  the script stopped: $($_.Exception.Message) (line $($_.InvocationInfo.ScriptLineNumber))"
    $checks.Add([pscustomobject]@{ Name = 'the script ran to its end'; Ok = $false; Detail = "$($_.Exception.Message) (line $($_.InvocationInfo.ScriptLineNumber))" }) | Out-Null
    Finish 1
    # A stopped run must not leave an install behind: uninstall it before the work folder goes.
    $u = Join-Path $dir 'unins000.exe'
    if (Test-Path -LiteralPath $u) {
        Start-Process -FilePath $u -ArgumentList '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART' -Wait | Out-Null
        $stop = (Get-Date).AddSeconds(90)
        while ((Test-Path -LiteralPath $u) -and (Get-Date) -lt $stop) { Start-Sleep -Milliseconds 500 }
    }
    Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue
    exit 1
}

Say "package-test: setup $Setup"
Say "package-test: sha256 $sha256 ($setupBytes bytes)"
Say "package-test: into  $dir"
Say "package-test: ticket $Ticket"

# Whatever is already installed would make the checks meaningless.
$before = @(UninstallKeys)
if ($before) { Say 'package-test: a Spade Editor is already installed for this user; uninstall it first'; Remove-Item -Recurse -Force $work; exit 2 }

# ---- 1 install ------------------------------------------------------------------------------------------------------------------------
$installLog = Join-Path $work 'install.log'
$p = Start-Process -FilePath $Setup -ArgumentList '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', '/CURRENTUSER', '/TASKS=desktopicon', "`"/DIR=$dir`"", "`"/LOG=$installLog`"" -Wait -PassThru
Check 'install exits 0' ($p.ExitCode -eq 0) "(exit $($p.ExitCode); log $installLog)"
$keys = @(UninstallKeys)
Check 'uninstaller registered under HKCU only (per-user, no administrator)' ($keys.Count -eq 1 -and $keys[0] -like '*HKEY_CURRENT_USER*') "($($keys -join '; '))"

# ---- 2 file list ----------------------------------------------------------------------------------------------------------------------
$must = 'spade_sandbox.exe', 'spade_geo_fetch.exe', 'spade_part_import.exe', 'step_convert.py', 'LICENSE', 'THIRD_PARTY_NOTICES.md', 'VERSION', 'INSTALL.md', 'RELEASE_NOTES.md', 'SYSTEM_REQUIREMENTS.md',
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
$versionPrinted = "exit $($v.Code): $($v.Output)"
Check "--version prints $expectVersion" ($v.Code -eq 0 -and $v.Output -like "*$expectVersion*") "(exit $($v.Code): '$($v.Output)')"

# ---- 4b the map fetch tool starts from the install folder, and with no network and an empty cache says miss (exit 3) -----------------------
$geoExe = Join-Path $dir 'spade_geo_fetch.exe'
$geoHelp = Run $geoExe @('--help') $dir
Check 'spade_geo_fetch --help exits 0 and prints its usage' ($geoHelp.Code -eq 0 -and $geoHelp.Output -like '*usage: spade_geo_fetch*') "(exit $($geoHelp.Code): '$($geoHelp.Output)')"
$geoCache = Join-Path $env:TEMP "spade-geo-check-$PID"
$geoCheck = Run $geoExe @('--lat', '40.76', '--lon', '-113.90', '--check', '--cache', $geoCache) $dir
Check 'spade_geo_fetch --check on an empty cache is a miss (exit 3), without the network' ($geoCheck.Code -eq 3 -and $geoCheck.Output -like '*miss*') "(exit $($geoCheck.Code): '$($geoCheck.Output)')"
Remove-Item -LiteralPath $geoCache -Recurse -Force -ErrorAction SilentlyContinue

# ---- 4c the part import tool starts from the install folder; with no python venv beside it, --probe says STEP is unavailable (exit 3) ----------
$piExe = Join-Path $dir 'spade_part_import.exe'
$piHelp = Run $piExe @('--help') $dir
Check 'spade_part_import --help exits 0 and prints its usage' ($piHelp.Code -eq 0 -and $piHelp.Output -like '*usage: spade_part_import*') "(exit $($piHelp.Code): '$($piHelp.Output)')"
$piProbe = Run $piExe @('--probe') $dir
Check 'spade_part_import --probe answers one JSON result line (exit 0 with a converter, exit 3 without one)' (($piProbe.Code -eq 0 -or $piProbe.Code -eq 3) -and $piProbe.Output -like '*"event":"result"*') "(exit $($piProbe.Code): '$($piProbe.Output)')"

# ---- 5 live smoke from the install folder ---------------------------------------------------------------------------------------------
if ($SkipSmoke) {
    Say 'package-test: SKIP  live smoke (-SkipSmoke)'
} else {
    $smokeRan = $true
    $s = Run $exe @('--live-smoke', '--out', $SmokeOut, '--label', "install-test $expectVersion", '--assets', (Join-Path $dir 'assets')) $dir
    $journal = Join-Path $SmokeOut 'journal.txt'
    $lines = if (Test-Path -LiteralPath $journal) { @(Get-Content -LiteralPath $journal -Encoding UTF8 | Where-Object { $_ }) } else { @() }
    $last = if ($lines.Count -gt 0) { $lines[$lines.Count - 1] } else { '(no journal)' }
    if ($lines.Count -gt 0) {
        # Keep the journal, the tour video and the numbered chapter stills beside the setup: the work folder goes at the end of the run.
        $keptDir = "$recordBase.smoke"
        New-Item -ItemType Directory -Force -Path $keptDir | Out-Null
        $toKeep = @(Get-ChildItem -LiteralPath $SmokeOut -File -ErrorAction SilentlyContinue |
            Where-Object { $_.Name -eq 'journal.txt' -or $_.Extension -eq '.mp4' -or $_.Name -match '^\d\d_.*\.png$' })
        foreach ($f in $toKeep) { Copy-Item -LiteralPath $f.FullName -Destination (Join-Path $keptDir $f.Name) -Force }
        $smokeInfo = @{
            Exit = $s.Code; Journal = $journal; End = $last
            Result = (@($lines | Where-Object { $_ -like 'RESULT *' }) | Select-Object -Last 1)
            ChecksCounted = @($lines | Where-Object { $_ -like 'CHECK *' }).Count
            Notes = @($lines | Where-Object { $_ -match '^NOTE (chapter|label)' } | Select-Object -First 12)
            KeptDir = $keptDir; Kept = @($toKeep | ForEach-Object Name)
        }
    }
    Check 'live smoke from the install folder: exit 0, END verdict=PASS' ($s.Code -eq 0 -and $last -eq 'END verdict=PASS') "(exit $($s.Code); journal ends '$last'; $journal)"
}

# ---- 6 uninstall ----------------------------------------------------------------------------------------------------------------------
if ($Keep) {
    Say "package-test: KEPT  the install at $dir (-Keep); uninstall with: `"$dir\unins000.exe`""
    Say 'package-test: step 6 (uninstall) not run'
    Finish 3 $true
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
    Say "package-test: ALL PASS ($expectVersion)"
    Finish 0
    exit 0
}
Say "package-test: FAILED: $($failed -join '; ')"
Finish 1
exit 1
