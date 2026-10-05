<#
.SYNOPSIS
    Records the sandbox's live smoke: a scripted tour of spade_sandbox in its
    window, saved as an MP4 with a journal and a verdict.

.DESCRIPTION
    Builds spade_sandbox (release) unless -NoBuild, runs
    `spade_sandbox --live-smoke`, and prints the video path and the verdict.

    The run writes to <OutRoot>\<UTC date>_<commit sha12>\:
      tour.mp4      the tour, 30 fps, with a title card at each end
      NN_<main>.png one still per main function (drone_box, builder)
      journal.txt   every step, check and anomaly, as it happened, ending
                    with "END verdict=PASS|FAIL"
    The sandbox removes only those names from the folder before writing, so
    a stale frame never poses as a new one.

    It needs a display, so it is not part of ctest; the verdict machinery is
    (tests/test_sandbox_live_smoke.cpp). ffmpeg must be on the PATH for the
    MP4. Without it the run records an anomaly and keeps PNG stills.

    Exit code: 0 PASS, 1 FAIL, 2 no verdict (the build failed, no window
    opened, or the journal has no END line because the run did not finish).

.PARAMETER OutRoot
    The folder the dated run folder goes under. Default: spade-smoke on the
    Desktop.

.PARAMETER NoBuild
    Run the existing build-ninja\release\bin\spade_sandbox.exe as it is.

.PARAMETER Inject
    For red runs only, passed through as --inject: skip:<step>,
    anomaly:<id> or known-open:<id>. Each must make the run FAIL.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\live-smoke.ps1
#>
param(
    [string] $OutRoot = (Join-Path ([Environment]::GetFolderPath('Desktop')) 'spade-smoke'),
    [switch] $NoBuild,
    [string[]] $Inject = @()
)
$ErrorActionPreference = 'Stop'

$repo = Split-Path -Parent $PSScriptRoot
$sha = ''
try {
    $sha = (& git -C $repo rev-parse --short=12 HEAD).Trim()
} catch {
    Write-Host "live-smoke: git did not answer, so the folder and the title card have no commit: $_"
}
if (-not $sha) { $sha = 'nocommit' }
$date = (Get-Date).ToUniversalTime().ToString('yyyy-MM-dd')
$out = Join-Path $OutRoot "${date}_$sha"

if (-not $NoBuild) {
    & (Join-Path $PSScriptRoot 'build.ps1') -Preset release -Target spade_sandbox -ParallelLevel 8
    if ($LASTEXITCODE -ne 0) {
        Write-Host "live-smoke: the spade_sandbox build failed (exit $LASTEXITCODE); fix it, or pass -NoBuild to run the last build"
        exit 2
    }
}

$exe = Join-Path $repo 'build-ninja\release\bin\spade_sandbox.exe'
if (-not (Test-Path $exe)) {
    Write-Host "live-smoke: $exe does not exist; run without -NoBuild to build it"
    exit 2
}

$runArgs = @('--live-smoke', '--out', $out, '--label', "$sha $date")
foreach ($spec in $Inject) { $runArgs += @('--inject', $spec) }
Write-Host "live-smoke: $exe $($runArgs -join ' ')"
& $exe @runArgs
$code = $LASTEXITCODE

$journal = Join-Path $out 'journal.txt'
$last = ''
if (Test-Path $journal) {
    $last = Get-Content $journal | Where-Object { $_ } | Select-Object -Last 1
}
Write-Host ''
if ($last -notlike 'END verdict=*') {
    Write-Host "live-smoke: UNFINISHED -- the journal has no END line, so the run stopped before its verdict (exit $code)."
    Write-Host "live-smoke: journal $journal"
    exit 2
}
Write-Host "live-smoke: $last (exit $code)"
$mp4 = Join-Path $out 'tour.mp4'
if (Test-Path $mp4) {
    Write-Host "live-smoke: video   $mp4"
} else {
    Write-Host 'live-smoke: video   none -- see the journal for why'
}
Write-Host "live-smoke: journal $journal"
exit $code
