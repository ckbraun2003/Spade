<#
.SYNOPSIS
    Runs the autopilot tests (ctest label `fc`) against a PX4 SITL container, as one queued job: Docker up, one container for the run, the tests, the
    container stopped, a summary. Wraps Interface's scripts\px4-sitl.ps1; it does not replace it.

.DESCRIPTION
    The tests that need a real autopilot carry the ctest label `fc` and not `spade` (TD-17), so scripts\test.ps1 and the Docker leg never run them. This is
    how they run (docs/design/test-docs/plans/2026-11-21-fc-tests.md, section 2):
      1  scripts\docker-up.ps1: the daemon answers (Kat quits Docker Desktop when idle). A daemon that never answers: exit 1.
      2  scripts\px4-sitl.ps1 -RunId <id> -HostAddress <addr> -OutDir <run folder>: starts ONE container, spade-px4-<id>. Interface owns that script, the image
         (spade-px4-sitl:v1.17.0), the ports and the airframe file. Its exits (b558905e): 2 no image, 3 Docker unreachable, 4 container not up (its log
         follows the first line), 5 run id in use. 2 is NOT GREEN, exit 2: the tests are listed by name as not run. 3, 4 and 5 are FAIL, exit 1, each named,
         the tests not run. Nothing is stopped after a failed start. A skipped test never counts toward green (TD-13's reading).
      3  `ctest -L fc` in the build tree, with SPADE_FC_RUN_ID, SPADE_FC_HOST_ADDR and SPADE_FC_OUT_DIR set for the tests. The tests find the autopilot through
         those; with them unset a test skips by name ("no autopilot run").
      4  scripts\px4-sitl.ps1 -RunId <id> -Stop -OutDir <run folder>, always, once step 2 started the container, whatever the tests did. It never stops
         Docker Desktop. Its LAST line is the bare newest .ulg path, or `none` (PX4 wrote no log: said in the summary, not a failure). Exit 3 (Docker
         unreachable: the container may still be running) and 6 (no run folder known) are FAIL; a "no container named" line (it was gone) is NOT GREEN.
      5  <run folder>\summary.txt: `fc run: PASS` or `fc run: FAIL` or `fc run: NOT GREEN`, the run id, the commit, the ctest counts, every skipped test by
         name, the autopilot log's path (the last line px4-sitl.ps1 printed when it stopped, relative to the run folder), and the seconds. What this script
         writes into the run folder (summary.txt, ctest.log, ctest-junit.xml) has the user's folders redacted, as the installer records do (K10).
    Exit: 0 only when ctest exited 0, at least one test ran, none skipped and the container was there at the end; 1 a failed test, a missing ctest or build
    tree, a daemon that never answered, or a start or stop failure other than "no image" (px4-sitl.ps1 exit 3, 4, 5, 6 or an unnamed one);
    2 not green without a failure (no image, a test skipped, or the container gone when the tests ended).

    Queue it, never run it bare (JOBS.md): `--class test --floor-gb 4`, a ticket with `-fc-` in it, and a `--limit-s` of at least 300 s plus the run, because
    docker-up.ps1 waits up to 300 s for a cold daemon.

.PARAMETER RunId
    Names the container and the run folder. Default: fc-<UTC date and time>.

.PARAMETER Build
    The configured build tree that holds spade_tests (its CTestTestfile). Default: build-ninja\release in the repository.

.PARAMETER OutRoot
    Where the run folder goes. Default: build-fc in the repository (git ignores build-*).

.PARAMETER HostAddress
    What PX4, inside its container, connects to: the machine that runs Spade's tests. Default: host.docker.internal (Docker Desktop's host alias).

.PARAMETER Filter
    A ctest -R regex that narrows the `fc` tests. Default: every one.

.PARAMETER DockerUp
    The script that makes the daemon answer. Default: scripts\docker-up.ps1. A test points it at a fake.

.PARAMETER Px4Script
    Interface's start and stop script. Default: scripts\px4-sitl.ps1. A test points it at a fake.

.PARAMETER PublishOffboard
    Passed on to px4-sitl.ps1 as its -PublishOffboard (the container's UDP 14580 on this machine's loopback only). Nothing else changes.

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File scripts\fc-run.ps1 -Build build-ninja\release
#>
[CmdletBinding()]
param(
    [string] $RunId = ('fc-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')),
    [string] $Build,
    [string] $OutRoot,
    [string] $HostAddress = 'host.docker.internal',
    [string] $Filter = '',
    [string] $DockerUp,
    [string] $Px4Script,
    [switch] $PublishOffboard
)
# 'Continue', not 'Stop': Windows PowerShell 5.1 turns a native command's stderr under 2>&1 into a terminating error, and ctest and docker write there.
$ErrorActionPreference = 'Continue'
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not $Build) { $Build = Join-Path $Repo 'build-ninja\release' }
if (-not $OutRoot) { $OutRoot = Join-Path $Repo 'build-fc' }
if (-not $DockerUp) { $DockerUp = Join-Path $PSScriptRoot 'docker-up.ps1' }
if (-not $Px4Script) { $Px4Script = Join-Path $PSScriptRoot 'px4-sitl.ps1' }
if ($RunId -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$') { Write-Host "fc-run: RunId '$RunId' is not a plain name"; exit 1 }

$clock = [Diagnostics.Stopwatch]::StartNew()
$out = Join-Path $OutRoot $RunId
New-Item -ItemType Directory -Force -Path $out | Out-Null
$summary = Join-Path $out 'summary.txt'
$ps = Join-Path $PSHOME 'powershell.exe'

# What goes into the run folder is shareable (the checker's K10: no user folder in any text file), so the files this script writes pass through the
# installer records' redaction (scripts\package-test-record.ps1): the profile, local-app-data and roaming folders become %USERPROFILE%, %LOCALAPPDATA%
# and %APPDATA%. The console keeps the real paths. The run folder's own files (the tests', PX4's) are theirs to keep clean.
. (Join-Path $PSScriptRoot 'package-test-record.ps1')

function Get-Commit {
    $c = ''
    try { $c = (& git -C $Repo rev-parse --short=12 HEAD 2>$null | Select-Object -First 1) } catch { $c = '' }
    if ($c) { return "$c" } else { return '(no git)' }
}

# One place that writes the verdict, the summary file and the exit code.
function Finish([string] $Verdict, [int] $Code, [string[]] $Lines) {
    $text = @("fc run: $Verdict", "run: $RunId", "commit: $(Get-Commit)", "seconds: $([int] $clock.Elapsed.TotalSeconds)") + $Lines
    Set-Content -LiteralPath $summary -Value @($text | ForEach-Object { ConvertTo-PublicRecord $_ }) -Encoding ASCII
    $text | ForEach-Object { Write-Host "fc-run: $_" }
    exit $Code
}

# 1: the daemon.
& $ps -NoProfile -ExecutionPolicy Bypass -File $DockerUp
if ($LASTEXITCODE -ne 0) { Finish 'FAIL' 1 @('the Docker daemon did not answer (scripts\docker-up.ps1 failed); nothing was started') }

if (-not (Test-Path -LiteralPath (Join-Path $Build 'CTestTestfile.cmake'))) {
    Finish 'FAIL' 1 @("no configured build tree at $Build (no CTestTestfile.cmake)")
}
if (-not (Get-Command ctest -ErrorAction SilentlyContinue)) { Finish 'FAIL' 1 @('ctest is not on PATH') }

# 2: the container. The last line it printed is the reason when it fails.
$px4Extra = @(if ($PublishOffboard) { '-PublishOffboard' })  # always an array: a one-element array out of a bare if is unrolled to a string, and splatting a string fails
$px4Out = @(& $ps -NoProfile -ExecutionPolicy Bypass -File $Px4Script -RunId $RunId -HostAddress $HostAddress -OutDir $out @px4Extra 2>&1 | ForEach-Object { "$_" })
$px4Code = $LASTEXITCODE
if ($px4Code -ne 0) {
    $px4Lines = @($px4Out | Where-Object { $_.Trim() })
    $reason = if ($px4Lines.Count -gt 0) { $px4Lines[0] } else { 'no output' }
    # px4-sitl.ps1's start exits (b558905e): 2 no image, 3 docker unreachable, 4 not up (its log follows the first line), 5 run id in use. Only
    # "no image" is NOT GREEN (the tests are listed as not run); the rest are failures, each named. Nothing is stopped: the container is not ours
    # (5) or px4-sitl.ps1 removed it itself (4).
    if ($px4Code -eq 2) {
        $names = @(& ctest --test-dir $Build -N -L fc 2>&1 | ForEach-Object { "$_" } | Where-Object { $_ -match '^\s*Test\s+#\d+:\s+(\S+)' } | ForEach-Object { ($_ -replace '^\s*Test\s+#\d+:\s+', '').Trim() })
        $lines = @("no autopilot image: $($px4Lines[-1])", "tests not run: $($names.Count)") + @($names | ForEach-Object { "not run: $_" })
        Finish 'NOT GREEN' 2 $lines
    }
    $why = switch ($px4Code) {
        3 { 'Docker was not reachable when px4-sitl.ps1 started (docker-up.ps1 had said it was up); nothing was started' }
        4 { "the PX4 container did not stay up (px4-sitl.ps1 removed it)" }
        5 { "the run id is already in use: a container spade-px4-$RunId exists; choose another -RunId (that container was not touched)" }
        default { "px4-sitl.ps1 exited $px4Code, which its contract does not name" }
    }
    $rest = if ($px4Lines.Count -gt 1) { @($px4Lines[1..($px4Lines.Count - 1)] | Select-Object -Last 8) } else { @() }
    $tail = @($rest | ForEach-Object { "  | $_" })
    Finish 'FAIL' 1 (@($why, "px4-sitl.ps1 said: $reason") + $tail + @('the tests were not run'))
}

# 3 and 4: the tests, then the container stopped whatever they did.
$env:SPADE_FC_RUN_ID = $RunId
$env:SPADE_FC_HOST_ADDR = $HostAddress
$env:SPADE_FC_OUT_DIR = $out
$ctestArgs = @('--test-dir', $Build, '-L', 'fc', '--output-on-failure', '--no-tests=error', '--output-junit', (Join-Path $out 'ctest-junit.xml'))
if ($Filter) { $ctestArgs += @('-R', $Filter) }
$ctestOut = @()
$ctestCode = 1
try {
    $ctestOut = @(& ctest @ctestArgs 2>&1 | ForEach-Object { "$_" })
    $ctestCode = $LASTEXITCODE
} finally {
    $stopOut = @(& $ps -NoProfile -ExecutionPolicy Bypass -File $Px4Script -RunId $RunId -Stop -OutDir $out 2>&1 | ForEach-Object { "$_" })
    $stopCode = $LASTEXITCODE
    Remove-Item Env:SPADE_FC_RUN_ID, Env:SPADE_FC_HOST_ADDR, Env:SPADE_FC_OUT_DIR -ErrorAction SilentlyContinue
}
Set-Content -LiteralPath (Join-Path $out 'ctest.log') -Value @($ctestOut | ForEach-Object { ConvertTo-PublicRecord $_ }) -Encoding ASCII
# ctest wrote the junit file itself, with the build tree and the test paths in it: the same redaction, in place.
$junitPath = Join-Path $out 'ctest-junit.xml'
if (Test-Path -LiteralPath $junitPath) {
    $utf8 = New-Object System.Text.UTF8Encoding($false)
    [IO.File]::WriteAllText($junitPath, (ConvertTo-PublicRecord ([IO.File]::ReadAllText($junitPath, $utf8))), $utf8)
}

# 5: the summary. A skipped test is named and never counts toward green.
$skipped = @($ctestOut | Where-Object { $_ -match '\(Skipped\)' -or $_ -match '\*\*\*Skipped' } | ForEach-Object {
        if ($_ -match 'Test\s+#\d+:\s+(\S+)') { $Matches[1] } elseif ($_ -match '^\s*\d+\s*-\s*(\S+)') { $Matches[1] } else { $_.Trim() } })
$counts = ($ctestOut | Where-Object { $_ -match 'tests passed, \d+ tests failed out of \d+' } | Select-Object -Last 1)
# px4-sitl.ps1 -Stop (b558905e): its LAST line is the bare newest .ulg path, or `none` when PX4 wrote no log; "no container named" before it means
# the container was gone; exit 3 docker unreachable (the container may still be running), 6 no run folder known.
$stopLines = @($stopOut | Where-Object { $_.Trim() })
$last = if ($stopLines.Count -gt 0) { $stopLines[-1].Trim() } else { '' }
$ulg = if (-not $last) { '(px4-sitl.ps1 -Stop printed nothing)' } elseif ($last -eq 'none') { 'none (PX4 wrote no .ulg)' } else { $last }
# The log's path is given relative to the run folder (it lives in it): the summary travels with the folder and names no host folder.
$outFull = [IO.Path]::GetFullPath($out).TrimEnd('\', '/')
if ($ulg.Length -gt $outFull.Length + 1 -and $ulg.StartsWith($outFull, [StringComparison]::OrdinalIgnoreCase) -and ($ulg[$outFull.Length] -eq '\' -or $ulg[$outFull.Length] -eq '/')) {
    $ulg = $ulg.Substring($outFull.Length + 1)
}
$gone = @($stopLines | Where-Object { $_ -like '*no container named*' }).Count -gt 0
$lines = @("ctest exit: $ctestCode", $(if ($counts) { "ctest: $counts" } else { 'ctest: no summary line' }), "tests skipped: $($skipped.Count)") +
    @($skipped | ForEach-Object { "skipped: $_" }) + @("autopilot log: $ulg")
$stopBad = $false
if ($stopCode -eq 3) {
    $stopBad = $true
    $lines += "Docker was not reachable at -Stop: the container spade-px4-$RunId may still be running; remove it with: docker rm -f spade-px4-$RunId"
} elseif ($stopCode -eq 6) {
    $stopBad = $true
    $lines += 'px4-sitl.ps1 -Stop: no run folder known, so the PX4 log was not collected'
} elseif ($stopCode -ne 0) {
    $stopBad = $true
    $lines += "px4-sitl.ps1 -Stop exited $stopCode, which its contract does not name"
}
if ($gone) { $lines += "the container spade-px4-$RunId was gone when the tests ended: PX4 was not running at the end of the run" }
if ($ctestCode -ne 0 -or $stopBad) { Finish 'FAIL' 1 $lines }
if ($skipped.Count -gt 0) { Finish 'NOT GREEN' 2 ($lines + @('a skipped test does not count toward green')) }
if ($gone) { Finish 'NOT GREEN' 2 $lines }
Finish 'PASS' 0 $lines
