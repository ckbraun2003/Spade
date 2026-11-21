<#
.SYNOPSIS
  Films one fc run (scripts\fc-run.ps1 with the FcLivePx4 test) as one raw capture: PX4's console on the left, Spade's lockstep side on the right.

.DESCRIPTION
  O150: each feature that lands gets a short video. This is the capture half for the PX4 link's first real run. It opens two console windows of its own, so
  nothing else on the desktop is filmed (ffmpeg's gdigrab takes each window by its title):
    "PX4 SITL v1.17.0 - airframe 10090 (Docker)"   `docker logs -f` of the run's container, from the first PX4 line
    "Spade lockstep link"                         the test's progress lines (every 250 steps: step, simulated and wall time, gaps by name) and, when the
                                                  run ends, the whole lockstep-report.txt
  ffmpeg starts when the container is up and stops a few seconds after the report is on screen. The raw file is <run folder>\film-raw.mp4 (H.264, the two windows
  side by side at 720 p high, 15 fps); the lead encodes it with check_video.py.

  This script is the job: queue it like fc-run.ps1 (a ticket with -fc-, class test). It runs fc-run.ps1 as a child and returns fc-run's exit code. A film
  that fails (no ffmpeg, no window) does not fail the run: the run's own result stands and the film's problem is printed on one line.

.PARAMETER RunId   Names the container and the run folder (same as fc-run.ps1). Default fc-<UTC date and time>.
.PARAMETER Filter  The ctest -R regex handed to fc-run.ps1. Default FcLivePx4.
.PARAMETER Build   Handed to fc-run.ps1.
.PARAMETER OutRoot Handed to fc-run.ps1 (default build-fc in the repository).
#>
[CmdletBinding()]
param(
    [string] $RunId = ('fc-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')),
    [string] $Filter = 'FcLivePx4',
    [string] $Build,
    [string] $OutRoot
)
$ErrorActionPreference = 'Continue'
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
if (-not $OutRoot) { $OutRoot = Join-Path $Repo 'build-fc' }
if ($RunId -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]*$') { Write-Host "fc-film: RunId '$RunId' is not a plain name"; exit 1 }
$out = Join-Path $OutRoot $RunId
New-Item -ItemType Directory -Force -Path $out | Out-Null
$ps = Join-Path $PSHOME 'powershell.exe'
$container = "spade-px4-$RunId"
$titleA = 'PX4 SITL v1.17.0 - airframe 10090 (Docker)'
$titleB = 'Spade lockstep link'
$progress = Join-Path $out 'lockstep-progress.txt'
$report = Join-Path $out 'lockstep-report.txt'
$raw = Join-Path $out 'film-raw.mp4'
Remove-Item -ErrorAction SilentlyContinue $progress, $report, $raw

function Start-FilmWindow([string] $title, [string] $body) {
    $script = "`$Host.UI.RawUI.WindowTitle = '$title'; try { `$Host.UI.RawUI.BufferSize = New-Object Management.Automation.Host.Size(110, 3000); `$Host.UI.RawUI.WindowSize = New-Object Management.Automation.Host.Size(110, 38) } catch {}; $body"
    $enc = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($script))
    return Start-Process -FilePath $ps -ArgumentList @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-EncodedCommand', $enc) -PassThru
}

# window A: PX4's own console output, from the moment the container exists
$bodyA = "while (-not (docker ps -q --filter 'name=^/$container`$')) { Start-Sleep -Milliseconds 200 }; docker logs -f $container; Start-Sleep -Seconds 6"
# window B: the lockstep side; ends 8 s after the report is written
$bodyB = "`$seen = 0; while (-not (Test-Path '$report')) { if (Test-Path '$progress') { `$l = @(Get-Content '$progress'); while (`$seen -lt `$l.Count) { `$l[`$seen]; `$seen++ } }; Start-Sleep -Milliseconds 300 }; ''; '--- lockstep report ---'; Get-Content '$report'; Start-Sleep -Seconds 8"
$winA = Start-FilmWindow $titleA $bodyA
$winB = Start-FilmWindow $titleB $bodyB

# the run itself
$runArgs = @('-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', (Join-Path $PSScriptRoot 'fc-run.ps1'), '-RunId', $RunId, '-Filter', $Filter, '-OutRoot', $OutRoot)
if ($Build) { $runArgs += @('-Build', $Build) }
$runProc = Start-Process -FilePath $ps -ArgumentList $runArgs -PassThru -NoNewWindow

# the film: wait for the container, then both windows by title
$ffmpeg = Get-Command ffmpeg -ErrorAction SilentlyContinue
$ff = $null
$filmNote = ''
if (-not $ffmpeg) { $filmNote = 'no ffmpeg on the PATH' }
else {
    $deadline = [DateTime]::UtcNow.AddSeconds(420)
    while (-not (docker ps -q --filter "name=^/$container$") -and [DateTime]::UtcNow -lt $deadline -and -not $runProc.HasExited) { Start-Sleep -Milliseconds 200 }
    if (-not (docker ps -q --filter "name=^/$container$")) { $filmNote = 'the container never came up while filming' }
    else {
        Start-Sleep -Milliseconds 800  # the windows exist by now; give them their titles and size
        $psi = New-Object Diagnostics.ProcessStartInfo
        $psi.FileName = $ffmpeg.Source
        $psi.Arguments = "-y -loglevel warning -f gdigrab -framerate 15 -i title=`"$titleA`" -f gdigrab -framerate 15 -i title=`"$titleB`" -filter_complex `"[0:v]scale=-2:720[a];[1:v]scale=-2:720[b];[a][b]hstack,pad=ceil(iw/2)*2:ceil(ih/2)*2`" -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p `"$raw`""
        $psi.RedirectStandardInput = $true
        $psi.UseShellExecute = $false
        $ff = [Diagnostics.Process]::Start($psi)
    }
}

$runProc.WaitForExit()
$code = $runProc.ExitCode
# the lockstep window ends itself 8 s after the report; give it that, then stop ffmpeg
$null = $winB.WaitForExit(40000)
if ($ff -and -not $ff.HasExited) {
    try { $ff.StandardInput.WriteLine('q') } catch {}
    if (-not $ff.WaitForExit(20000)) { $ff.Kill(); $filmNote = 'ffmpeg had to be killed; the file may be cut' }
}
foreach ($w in @($winA, $winB)) { if ($w -and -not $w.HasExited) { $w.Kill() } }
if (Test-Path $raw) { Write-Host ("fc-film: raw capture {0} ({1} bytes)" -f $raw, (Get-Item $raw).Length) }
else { Write-Host ("fc-film: no capture ({0}); the run's own result stands" -f $(if ($filmNote) { $filmNote } else { 'ffmpeg wrote nothing' })) }
if ($filmNote -and (Test-Path $raw)) { Write-Host "fc-film: note: $filmNote" }
exit $code
