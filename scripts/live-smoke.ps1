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
    The sandbox deletes only those names (and its fallback stills) from the
    folder before writing, and refuses a folder that holds anything else.

    It needs a display, so it is not part of ctest; the verdict machinery is
    (tests/test_sandbox_live_smoke.cpp). ffmpeg must be on the PATH for the
    MP4. Without it the run records an anomaly and keeps PNG stills.

    Exit code: 0 PASS, 1 FAIL, 2 no verdict (the build failed, the sandbox
    refused its arguments or folder, no window opened, it crashed, or its
    journal has no END line).

.PARAMETER OutRoot
    The folder the dated run folder goes under. Default: spade-smoke on the
    Desktop.

.PARAMETER NoBuild
    Run the existing build-ninja\release\bin\spade_sandbox.exe as it is.

.PARAMETER Inject
    For red runs only, passed through as --inject: skip:<step> or
    anomaly:<step>/<name>. Each must make the run FAIL; one that could not
    is refused (exit 2). A comma list is split, because powershell -File
    passes `-Inject a,b` as one string.

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
    # build.ps1 throws on a failed build, so a failure is caught here and
    # reported as "no verdict", never as the tour's FAIL.
    try {
        & (Join-Path $PSScriptRoot 'build.ps1') -Preset release -Target spade_sandbox -ParallelLevel 8
        $buildCode = $LASTEXITCODE
    } catch {
        Write-Host "live-smoke: the spade_sandbox build failed: $_"
        Write-Host 'live-smoke: no verdict (exit 2). Fix the build, or pass -NoBuild to run the last build.'
        exit 2
    }
    if ($buildCode -ne 0) {
        Write-Host "live-smoke: the spade_sandbox build failed (exit $buildCode)."
        Write-Host 'live-smoke: no verdict (exit 2). Fix the build, or pass -NoBuild to run the last build.'
        exit 2
    }
}

$exe = Join-Path $repo 'build-ninja\release\bin\spade_sandbox.exe'
if (-not (Test-Path $exe)) {
    Write-Host "live-smoke: $exe does not exist; run without -NoBuild to build it"
    exit 2
}

# The journal is deleted first, so a run that stops before writing its own
# can never be judged by an earlier run's.
$journal = Join-Path $out 'journal.txt'
if (Test-Path -LiteralPath $journal) {
    Remove-Item -LiteralPath $journal
}

$runArgs = @('--live-smoke', '--out', $out, '--label', "$sha $date")
foreach ($spec in ($Inject | ForEach-Object { $_ -split ',' } | Where-Object { $_ })) {
    $runArgs += @('--inject', $spec.Trim())
}
Write-Host "live-smoke: $exe $($runArgs -join ' ')"
# A native command's stderr must not stop this script: the sandbox explains
# a refusal there, and its exit code is what decides.
$ErrorActionPreference = 'Continue'
& $exe @runArgs
$code = $LASTEXITCODE
$ErrorActionPreference = 'Stop'

Write-Host ''
if ($code -ne 0 -and $code -ne 1) {
    if ($code -ne 2) {
        Write-Host "live-smoke: spade_sandbox ended with code $code, which is not a verdict (a crash?)."
    }
    Write-Host 'live-smoke: no verdict (exit 2).'
    if (Test-Path -LiteralPath $journal) {
        Write-Host "live-smoke: journal $journal (unfinished)"
    }
    exit 2
}

$last = ''
if (Test-Path -LiteralPath $journal) {
    $last = Get-Content -LiteralPath $journal | Where-Object { $_ } | Select-Object -Last 1
}
$expected = if ($code -eq 0) { 'END verdict=PASS' } else { 'END verdict=FAIL' }
if ($last -ne $expected) {
    Write-Host "live-smoke: UNFINISHED -- spade_sandbox exited $code but the journal ends '$last', not '$expected'."
    Write-Host "live-smoke: no verdict (exit 2). Journal: $journal"
    exit 2
}
Write-Host "live-smoke: $last (exit $code)"
$mp4 = Join-Path $out 'tour.mp4'
if (Test-Path -LiteralPath $mp4) {
    Write-Host "live-smoke: video   $mp4"
} else {
    Write-Host 'live-smoke: video   none -- see the journal for why'
}
Write-Host "live-smoke: journal $journal"
exit $code
