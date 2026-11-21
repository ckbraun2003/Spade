<#
.SYNOPSIS
    Runs the autopilot tests (ctest label `fc`) against a PX4 SITL container, as one queued job: Docker up, one container for the run, the tests, the
    container stopped, a summary. Wraps Interface's scripts\px4-sitl.ps1; it does not replace it.

.DESCRIPTION
    The tests that need a real autopilot carry the ctest label `fc` and not `spade` (TD-17), so scripts\test.ps1 and the Docker leg never run them. This is
    how they run (docs/design/test-docs/plans/2026-11-21-fc-tests.md, section 2):
      1  scripts\docker-up.ps1: the daemon answers (Kat quits Docker Desktop when idle). A daemon that never answers: exit 1.
      2  scripts\px4-sitl.ps1 -RunId <id> -HostAddress <addr> -OutDir <run folder>: starts ONE container, spade-px4-<id>. Interface owns that script, the image
         (spade-px4-sitl:v1.17.0), the ports and the airframe file. A non-zero exit is "no autopilot image" (or another reason, its last line): the tests are NOT
         run, they are listed by name as not run, and the exit is 2. A skipped test never counts toward green (TD-13's reading).
      3  `ctest -L fc` in the build tree, with SPADE_FC_RUN_ID, SPADE_FC_HOST_ADDR and SPADE_FC_OUT_DIR set for the tests. The tests find the autopilot through
         those; with them unset a test skips by name ("no autopilot run").
      4  scripts\px4-sitl.ps1 -RunId <id> -Stop, always, once step 2 started the container, whatever the tests did. It never stops Docker Desktop.
      5  <run folder>\summary.txt: `fc run: PASS` or `fc run: FAIL` or `fc run: NOT GREEN`, the run id, the commit, the ctest counts, every skipped test by
         name, the autopilot log's path (the last line px4-sitl.ps1 printed when it stopped), and the seconds.
    Exit: 0 only when ctest exited 0, at least one test ran and none skipped; 1 a failed test, a missing ctest or build tree, or a daemon that never answered;
    2 not green without a failure (no image, or a test skipped).

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
    [string] $Px4Script
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

function Get-Commit {
    $c = ''
    try { $c = (& git -C $Repo rev-parse --short=12 HEAD 2>$null | Select-Object -First 1) } catch { $c = '' }
    if ($c) { return "$c" } else { return '(no git)' }
}

# One place that writes the verdict, the summary file and the exit code.
function Finish([string] $Verdict, [int] $Code, [string[]] $Lines) {
    $text = @("fc run: $Verdict", "run: $RunId", "commit: $(Get-Commit)", "seconds: $([int] $clock.Elapsed.TotalSeconds)") + $Lines
    Set-Content -LiteralPath $summary -Value $text -Encoding ASCII
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
$px4Out = @(& $ps -NoProfile -ExecutionPolicy Bypass -File $Px4Script -RunId $RunId -HostAddress $HostAddress -OutDir $out 2>&1 | ForEach-Object { "$_" })
$px4Code = $LASTEXITCODE
if ($px4Code -ne 0) {
    $reason = if ($px4Out.Count -gt 0) { $px4Out[-1] } else { 'no output' }
    $names = @(& ctest --test-dir $Build -N -L fc 2>&1 | ForEach-Object { "$_" } | Where-Object { $_ -match '^\s*Test\s+#\d+:\s+(\S+)' } | ForEach-Object { ($_ -replace '^\s*Test\s+#\d+:\s+', '').Trim() })
    $lines = @("no autopilot: px4-sitl.ps1 exited $px4Code ($reason)", "tests not run: $($names.Count)") + @($names | ForEach-Object { "not run: $_" })
    Finish 'NOT GREEN' 2 $lines
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
Set-Content -LiteralPath (Join-Path $out 'ctest.log') -Value $ctestOut -Encoding ASCII

# 5: the summary. A skipped test is named and never counts toward green.
$skipped = @($ctestOut | Where-Object { $_ -match '\(Skipped\)' -or $_ -match '\*\*\*Skipped' } | ForEach-Object {
        if ($_ -match 'Test\s+#\d+:\s+(\S+)') { $Matches[1] } elseif ($_ -match '^\s*\d+\s*-\s*(\S+)') { $Matches[1] } else { $_.Trim() } })
$counts = ($ctestOut | Where-Object { $_ -match 'tests passed, \d+ tests failed out of \d+' } | Select-Object -Last 1)
$ulg = if ($stopOut.Count -gt 0) { $stopOut[-1] } else { '(px4-sitl.ps1 -Stop printed nothing)' }
$lines = @("ctest exit: $ctestCode", $(if ($counts) { "ctest: $counts" } else { 'ctest: no summary line' }), "tests skipped: $($skipped.Count)") +
    @($skipped | ForEach-Object { "skipped: $_" }) + @("autopilot log: $ulg")
if ($stopCode -ne 0) { $lines += "px4-sitl.ps1 -Stop exited $stopCode" }
if ($ctestCode -ne 0) { Finish 'FAIL' 1 $lines }
if ($skipped.Count -gt 0) { Finish 'NOT GREEN' 2 ($lines + @('a skipped test does not count toward green')) }
Finish 'PASS' 0 $lines
