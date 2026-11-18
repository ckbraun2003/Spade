<#
.SYNOPSIS
    The shape of the installer test's record (scripts\package-test-record.ps1): every field the owner asked for is there for a pass, a fail
    and a run with no smoke, and the ledger row is one line with the right number of columns. No install, no display, no GPU.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tests\package_test_record_test.ps1 <path to scripts\package-test-record.ps1>
#>
param([Parameter(Mandatory)] [string] $RecordScript)
$ErrorActionPreference = 'Stop'
. $RecordScript

$problems = New-Object System.Collections.Generic.List[string]
function Expect([string] $Case, [bool] $Ok, [string] $What) { if (-not $Ok) { $problems.Add("${Case}: $What") | Out-Null } }
function Has([string] $Text, [string] $Needle) { $Text.Contains($Needle) }

function Fixture([bool] $SmokeRun, [bool] $BreakACheck, [bool] $Kept) {
    $checks = @(
        [pscustomobject]@{ Name = 'install exits 0'; Ok = $true; Detail = '(exit 0; log C:\x\install.log)' },
        [pscustomobject]@{ Name = 'installed files present'; Ok = (-not $BreakACheck); Detail = '(missing: spade_geo_fetch.exe | LICENSE)' },
        [pscustomobject]@{ Name = 'nothing left under %APPDATA%\Spade'; Ok = $true; Detail = '' }
    )
    $smoke = $null
    if ($SmokeRun) {
        $smoke = @{ Exit = 0; Journal = 'C:\w\smoke\journal.txt'; End = 'END verdict=PASS'; Result = 'RESULT anomalies=0 known_open_fired=0 unmarked=0 expired=0'
                    ChecksCounted = 160; Notes = @('NOTE chapter airflow at 41 s of video'); KeptDir = 'C:\b\Spade-Editor-0.1.0-abcdef0-setup.smoke'
                    Kept = @('journal.txt', 'tour.mp4', '01_drone_box.png', '02_airflow.png') }
    }
    @{
        StartedUtc = [datetime]'2026-11-17T20:15:30Z'; SetupPath = 'C:\b\Spade-Editor-0.1.0-abcdef0-setup.exe'; SetupName = 'Spade-Editor-0.1.0-abcdef0-setup'
        SetupBytes = 38123456; Sha256 = ('4d858df5' + ('0' * 56)); SetupCommit = 'abcdef0'; ScriptCommit = '12345678'
        ExpectVersion = '0.1.0+abcdef0'; VersionPrinted = 'exit 0: spade_sandbox 0.1.0+abcdef0'; Ticket = 'test-docs-installer-abcdef0'; ElapsedSeconds = 254.6
        Machine = @{ Windows = 'Microsoft Windows 10 Home 10.0.19045 (build 19045)'; Cpu = 'AMD Ryzen 7 3700X'; Memory = '15.9 GB'
                     Gpus = @('NVIDIA GeForce RTX 3060 Ti, 572.83 (nvidia-smi: name, driver)', 'NVIDIA GeForce RTX 3060 Ti, Windows driver 32.0.15.7283') }
        Checks = $checks; SmokeRun = $SmokeRun; Smoke = $smoke; Kept = $Kept; LogPath = 'C:\b\Spade-Editor-0.1.0-abcdef0-setup.test.log'; Exit = 0
    }
}

# A pass.
$pass = New-InstallTestRecord (Fixture $true $false $false)
$case = 'pass'
Expect $case ($pass -match '(?m)^# Installer test: Spade-Editor-0\.1\.0-abcdef0-setup$') 'title'
Expect $case (Has $pass '**Result: PASS** (3 of 3 checks passed)') 'result line'
foreach ($needle in @('2026-11-17 20:15:30Z', 'abcdef0', '12345678', '38123456 bytes', ('4d858df5' + ('0' * 56)), 'test-docs-installer-abcdef0', '4 min 14 s',
                      'Microsoft Windows 10 Home 10.0.19045', 'AMD Ryzen 7 3700X', 'RTX 3060 Ti, 572.83', 'Windows driver 32.0.15.7283',
                      '| 1 | PASS | install exits 0 |', '| 2 | PASS | installed files present |', 'END verdict=PASS', 'RESULT anomalies=0',
                      'Journal checks counted: 160', 'NOTE chapter airflow at 41 s of video', 'A still: `C:\b\Spade-Editor-0.1.0-abcdef0-setup.smoke\01_drone_box.png`',
                      'Spade-Editor-0.1.0-abcdef0-setup.test.log', '## Machine', '## Checks', '## Live smoke')) {
    Expect $case (Has $pass $needle) "missing: $needle"
}
Expect $case (-not $pass.Contains("`r")) 'a carriage return in the record'

# A fail: the failed check is named in the result and marked in the table, and a pipe in a detail does not break the table row.
$fail = New-InstallTestRecord (Fixture $true $true $false)
$case = 'fail'
Expect $case (Has $fail '**Result: FAIL** (2 of 3 checks passed; failed: installed files present)') 'result line'
Expect $case (Has $fail '| 2 | FAIL | installed files present | (missing: spade_geo_fetch.exe \| LICENSE) |') 'the failed row, pipe escaped'

# No smoke: the record says the installer was not smoked and does not call it a pass.
$none = New-InstallTestRecord (Fixture $false $false $false)
$case = 'no smoke'
Expect $case (Has $none '**Result: PASS-NO-SMOKE**') 'verdict'
Expect $case (Has $none 'not smoked') 'the not-smoked sentence'
Expect $case (Has $none 'Not run: -SkipSmoke was given') 'the smoke section'

# Kept.
$kept = New-InstallTestRecord (Fixture $true $false $true)
Expect 'kept' (Has $kept '**Result: KEPT**') 'verdict'

# The ledger row: one line, nine cells, the sha prefix, the verdict and the record link.
$row = New-InstallLedgerRow (Fixture $true $false $false) 'installer-records/Spade-Editor-0.1.0-abcdef0-setup.test.md'
$case = 'ledger row'
Expect $case ($row -notmatch "[\r\n]") 'more than one line'
Expect $case (($row -split '(?<!\\)\|').Count -eq 11) "cell count: $((($row -split '(?<!\\)\|').Count) - 2), expected 9"
Expect $case (Has $row '`4d858df50000`') 'sha prefix'
Expect $case (Has $row '**PASS** 3/3') 'verdict and counts'
Expect $case (Has $row '[record](installer-records/Spade-Editor-0.1.0-abcdef0-setup.test.md)') 'record link'
Expect $case (Has $row 'END verdict=PASS') 'smoke cell'
$header = ($InstallLedgerHeader -split "`n" | Where-Object { $_ -like '|*' } | Select-Object -First 1)
Expect $case (($header -split '\|').Count -eq 11) 'the ledger header has nine columns'

# The copy for the public repo: no home folder. A made-up user, so the test does not depend on the machine it runs on.
$case = 'public copy'
$folders = @{ USERPROFILE = 'C:\Users\Ada'; LOCALAPPDATA = 'C:\Users\Ada\AppData\Local'; APPDATA = 'C:\Users\Ada\AppData\Roaming' }
$raw = @(
    'Setup path | C:\Users\Ada\Desktop\Projects\spade-wt\core-2\build-package\x-setup.exe',
    'log C:\Users\Ada\AppData\Local\Temp\spade-install-test-1\install.log)',
    'Start menu: C:\Users\Ada\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Spade Editor',
    'json {"p":"C:\\Users\\Ada\\AppData\\Local\\Temp\\spade-install-test-1\\Spade Editor\\.venv-step/bin/python","q":"C:\\Users\\Ada\\Desktop\\x"}',
    'forward c:/users/ada/AppData/Local/Temp/z and lower-case c:\users\ada\desktop',
    'a neighbour C:\Users\Adam\Desktop and C:\Users\Ada-2\Desktop are another user',
    'nothing under C:\Program Files\Spade Editor'
) -join "`n"
$pub = ConvertTo-PublicRecord $raw $folders
Expect $case (-not ($pub -match '(?i)users[\\/]+ada(?![\w-])')) "a home folder is left: $pub"
Expect $case (Has $pub 'Setup path | %USERPROFILE%\Desktop\Projects\spade-wt\core-2\build-package\x-setup.exe') 'profile folder'
Expect $case (Has $pub 'log %LOCALAPPDATA%\Temp\spade-install-test-1\install.log)') 'local app data, not %USERPROFILE%\AppData\Local'
Expect $case (Has $pub 'Start menu: %APPDATA%\Microsoft\Windows\Start Menu\Programs\Spade Editor') 'roaming app data'
Expect $case (Has $pub '"p":"%LOCALAPPDATA%\\Temp\\spade-install-test-1\\Spade Editor\\.venv-step/bin/python","q":"%USERPROFILE%\\Desktop\\x"') 'doubled backslashes keep their form'
Expect $case (Has $pub 'forward %LOCALAPPDATA%/Temp/z and lower-case %USERPROFILE%\desktop') 'forward slashes and any case'
Expect $case (Has $pub 'C:\Users\Adam\Desktop and C:\Users\Ada-2\Desktop') 'a longer folder name is left alone'
Expect $case (Has $pub 'nothing under C:\Program Files\Spade Editor') 'a folder outside the profile is left alone'
Expect $case ((ConvertTo-PublicRecord $raw @{}) -eq $raw) 'no folders given: text unchanged'

# A whole record through it, with this machine's own folders as the paths: none of them survives, and the rest of the record does.
$mine = @($env:USERPROFILE, $env:LOCALAPPDATA, $env:APPDATA) | Where-Object { $_ }
if ($mine.Count -eq 3) {
    $fx = Fixture $true $false $false
    $fx.SetupPath = Join-Path $env:USERPROFILE 'Desktop\spade\build-package\Spade-Editor-0.1.0-abcdef0-setup.exe'
    $fx.LogPath = Join-Path $env:USERPROFILE 'Desktop\spade\build-package\Spade-Editor-0.1.0-abcdef0-setup.test.log'
    $fx.Checks = @([pscustomobject]@{ Name = 'install exits 0'; Ok = $true; Detail = "(log $(Join-Path $env:LOCALAPPDATA 'Temp\spade-install-test-1\install.log'))" },
                   [pscustomobject]@{ Name = 'Start menu'; Ok = $true; Detail = (Join-Path $env:APPDATA 'Microsoft\Windows\Start Menu\Programs\Spade Editor') })
    $rec = New-InstallTestRecord $fx
    $pubRec = ConvertTo-PublicRecord $rec
    Expect 'public record' (Has $rec $env:USERPROFILE) 'the fixture should print the profile folder before the conversion'
    foreach ($f in $mine) { Expect 'public record' (-not $pubRec.Contains($f)) "still prints $f" }
    Expect 'public record' (Has $pubRec '%USERPROFILE%\Desktop\spade\build-package\Spade-Editor-0.1.0-abcdef0-setup.exe') 'setup path'
    Expect 'public record' (Has $pubRec '%LOCALAPPDATA%\Temp\spade-install-test-1\install.log') 'check detail'
    Expect 'public record' (Has $pubRec '**Result: PASS** (2 of 2 checks passed)') 'the rest of the record is intact'
}

if ($problems.Count -eq 0) { Write-Host 'package_test_record_test: all cases hold'; exit 0 }
foreach ($p in $problems) { Write-Host "package_test_record_test: FAIL $p" }
exit 1
