<#
The Docker leg (TD-11, TD-12): build a commit of Spade on Linux with gcc-13,
CPU-only, Release; run ctest -L spade -LE gpu; then build and run tests/consumer
against an installed prefix with SPADE_VULKAN ON and OFF (scripts/consumer-smoke.sh).

  scripts\docker-leg.ps1                  # the whole leg on HEAD
  scripts\docker-leg.ps1 -Commit <rev>    # another commit
  scripts\docker-leg.ps1 -Step build      # one step: image|sync|configure|build|test|consumer
  scripts\docker-leg.ps1 -Follow          # re-attach to a running leg
  scripts\docker-leg.ps1 -Stop            # stop a running leg
  scripts\docker-leg.ps1 -Clean           # drop the volume: source, build trees, dependencies

It tests a COMMIT, never the working tree: uncommitted changes are not in the leg.
Every step except image first syncs that commit into the spade-docker-leg volume,
so the leg always builds what it names. The leg runs in a detached container named
spade-docker-leg: closing the terminal does not stop it, and running this script
again re-attaches instead of starting a second one, so a watcher may be stopped
(Ctrl+C, a tool timeout) and resumed with -Follow at no cost to the leg. Run it in
the foreground and stay with it; -Stop if you must leave.

Output: build-docker\<short-sha>\ (leg.log, build.log, build-errors.txt, ctest.log,
ctest-junit.xml, tests.txt, summary.txt). Exit code: the leg's (0 pass, 1 fail).
#>
param(
    [string] $Commit = 'HEAD',
    [ValidateSet('all', 'image', 'sync', 'configure', 'build', 'test', 'consumer')] [string] $Step = 'all',
    [string] $Memory = '3g',
    [ValidateRange(1, 64)] [int] $Jobs = 1,
    [int] $MinFreeGB = 4,
    [switch] $Follow,
    [switch] $Stop,
    [switch] $Clean
)
$ErrorActionPreference = 'Stop'
$Name = 'spade-docker-leg'
$Volume = 'spade-docker-leg'
$Repo = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path

# Native commands report failure through $LASTEXITCODE, never by throwing.
function Invoke-Checked([string] $What) {
    if ($LASTEXITCODE -ne 0) { throw "$What failed (exit $LASTEXITCODE)" }
}

function Get-FreeGB {
    # Docker Desktop keeps its disk image under LOCALAPPDATA.
    $drive = Get-PSDrive -Name $env:LOCALAPPDATA.Substring(0, 1)
    [math]::Round($drive.Free / 1GB, 1)
}

# State of the fixed-name container: '' if there is none, else 'running', 'exited', ...
function Get-LegState {
    (& docker ps -a --filter "name=^$Name`$" --format '{{.State}}') -join ''
}

function Get-LegLabels {
    $raw = (& docker ps -a --filter "name=^$Name`$" --format '{{.Labels}}') -join ''
    $labels = @{}
    foreach ($pair in ($raw -split ',')) {
        $kv = $pair -split '=', 2
        if ($kv.Count -eq 2) { $labels[$kv[0]] = $kv[1] }
    }
    $labels
}

# Streams the running leg's log, waits for it, removes it, prints the summary.
function Watch-Leg {
    $labels = Get-LegLabels
    Write-Host ("docker-leg: following the leg on {0}, step {1}" -f $labels['spade.commit'], $labels['spade.step'])
    # The last 100 lines, then live: a re-attach does not replay a whole build
    # (leg.log has everything). Out-Host keeps the log out of the return value.
    & docker logs --follow --tail 100 $Name | Out-Host
    $code = [int]((& docker wait $Name) -join '')
    # Two watchers both get here; whichever is second finds the removal under
    # way. Windows PowerShell turns redirected native stderr into errors under
    # 'Stop', hence the local 'Continue'.
    $ErrorActionPreference = 'Continue'
    & docker rm $Name 2>$null | Out-Null
    $ErrorActionPreference = 'Stop'
    $out = $labels['spade.out']
    $free = Get-FreeGB
    $summary = if ($out) { Join-Path $out 'summary.txt' } else { $null }
    if ($summary -and (Test-Path $summary) -and -not (Select-String -Path $summary -Pattern '^host ' -Quiet)) {
        Add-Content -Path $summary `
            -Value ("host        {0} GB free on {1} after the run" -f $free, $env:LOCALAPPDATA.Substring(0, 2))
    }
    Write-Host ("docker-leg: exit {0}; {1} GB free on the host; output in {2}" -f $code, $free, $out)
    return $code
}

# ---- an existing leg comes first ---------------------------------------------
$state = Get-LegState
if ($Stop) {
    if ($state -eq '') { Write-Host 'docker-leg: no leg to stop'; exit 0 }
    & docker stop $Name | Out-Null
    & docker rm $Name | Out-Null
    Write-Host 'docker-leg: stopped'
    exit 0
}
if ($state -eq 'running') {
    if (-not $Follow) {
        Write-Host 'docker-leg: a leg is already running; following it instead of starting another (-Commit and -Step are ignored)'
    }
    exit (Watch-Leg)
}
if ($state -ne '') {
    # A finished leg nobody waited for, e.g. after a closed terminal.
    Write-Host "docker-leg: found a finished leg ($state) that nobody collected"
    $code = Watch-Leg
    if ($Follow) { exit $code }
} elseif ($Follow) {
    Write-Host 'docker-leg: no leg is running'
    exit 0
}

if ($Clean) {
    if ((& docker volume ls -q --filter "name=^$Volume`$") -join '') {
        & docker volume rm $Volume | Out-Null
        Invoke-Checked 'docker volume rm'
    }
    Write-Host "docker-leg: volume $Volume removed"
    exit 0
}

# ---- the commit ------------------------------------------------------------------
$sha = ((& git -C $Repo rev-parse --verify "$Commit^{commit}") -join '').Trim()
Invoke-Checked "git rev-parse $Commit"
$dirty = @(& git -C $Repo status --porcelain --untracked-files=no).Count
if ($dirty -gt 0) {
    Write-Host "docker-leg: note: $dirty tracked file(s) have uncommitted changes; the leg tests $sha without them"
}

$free = Get-FreeGB
Write-Host ("docker-leg: {0} GB free on the host" -f $free)
if ($free -lt $MinFreeGB) {
    throw ("{0} GB free is under the {1} GB floor: a full disk truncates files mid-build (08-lessons). Free space, then run again" -f $free, $MinFreeGB)
}

$out = Join-Path $Repo ('build-docker\' + $sha.Substring(0, 12))
New-Item -ItemType Directory -Force -Path $out | Out-Null

# The repository's own bytes. A plain archive on a core.autocrlf=true box emits
# CRLF, which is not what a Linux checkout sees.
$tar = Join-Path $out 'src.tar'
& git -C $Repo -c core.autocrlf=false archive --format=tar -o $tar $sha
Invoke-Checked 'git archive'
[IO.File]::WriteAllText((Join-Path $out 'commit'), "$sha`n")

# The leg's own files come from the same archive, so they match the commit.
$ctx = Join-Path $out 'image'
if (Test-Path $ctx) { Remove-Item -Recurse -Force $ctx }
New-Item -ItemType Directory -Force -Path $ctx | Out-Null
# Windows' own tar (bsdtar), by path: launched from Git Bash, a bare `tar` is
# Git's GNU tar, which reads "C:\..." as a remote host and fails.
$bsdtar = Join-Path $env:SystemRoot 'System32\tar.exe'
if (-not (Test-Path $bsdtar)) { throw "$bsdtar not found (Windows 10 1803 or later ships it)" }
& $bsdtar -xf $tar -C $ctx scripts/docker-leg.Dockerfile scripts/docker-leg.sh
if ($LASTEXITCODE -ne 0) { throw "scripts/docker-leg.Dockerfile or scripts/docker-leg.sh is not in $sha" }
Copy-Item (Join-Path $ctx 'scripts\docker-leg.sh') (Join-Path $out 'docker-leg.sh') -Force
$dockerfile = Join-Path $ctx 'scripts\docker-leg.Dockerfile'

# ---- image: tagged by the Dockerfile's content -------------------------------------
$hash = (Get-FileHash -Algorithm SHA256 $dockerfile).Hash.ToLower().Substring(0, 12)
$image = "spade-docker-leg:$hash"
if (-not ((& docker images -q $image) -join '')) {
    Write-Host "docker-leg: building image $image"
    & docker build -t $image -f $dockerfile (Join-Path $ctx 'scripts')
    Invoke-Checked 'docker build'
} else {
    Write-Host "docker-leg: image $image is current"
}
if ($Step -eq 'image') { exit 0 }

# ---- sync, then the step ---------------------------------------------------------
$mounts = @('-v', "${Volume}:/leg", '-v', "${out}:/out")
& docker run --rm --name $Name @mounts $image bash /out/docker-leg.sh sync
Invoke-Checked 'sync'
if ($Step -eq 'sync') { exit 0 }

Write-Host "docker-leg: starting step $Step on $sha (memory $Memory, jobs $Jobs)"
& docker run --detach --name $Name --memory $Memory --memory-swap $Memory `
    --label "spade.commit=$sha" --label "spade.step=$Step" --label "spade.out=$out" `
    @mounts $image bash /out/docker-leg.sh $Step --jobs $Jobs | Out-Null
Invoke-Checked 'docker run'
exit (Watch-Leg)
