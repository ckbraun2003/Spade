<#
The Docker leg (TD-11, TD-12): build a commit of Spade on Linux with gcc-13,
CPU-only, Release; run ctest -L spade -LE gpu; then build and run tests/consumer
against an installed prefix with SPADE_VULKAN ON and OFF (scripts/consumer-smoke.sh).

  scripts\docker-leg.ps1                  # the whole leg on HEAD
  scripts\docker-leg.ps1 -Commit <rev>    # another commit
  scripts\docker-leg.ps1 -Run <id>        # name the run (default: the commit's short sha)
  scripts\docker-leg.ps1 -Step build      # one step: image|sync|configure|build|test|consumer
                                          # (consumer configures and builds first, so it
                                          #  installs the commit under test)
  scripts\docker-leg.ps1 -NoSeed          # start a new run's volume empty, not as a copy
  scripts\docker-leg.ps1 -Follow          # re-attach to the run's leg (or to the only leg)
  scripts\docker-leg.ps1 -Stop            # stop the run's leg (or the only leg)
  scripts\docker-leg.ps1 -Clean           # drop the run's volume
  scripts\docker-leg.ps1 -Prune [-Keep N] # drop idle leg volumes beyond the newest N (default 4)

It tests a COMMIT, never the working tree: uncommitted changes are not in the leg.
Each run has its own container and volume, both named spade-docker-leg-<run>. The
run defaults to the commit's short sha, so legs on different commits run side by
side, and the steps of one commit share a volume. A new volume starts as a copy of
the newest leg volume no running container uses (its build tree and source, mtimes
kept), so it neither re-fetches dependencies nor rebuilds what the commit did not
change; -NoSeed starts it empty. summary.txt says how the volume began.

Every step except image first syncs the commit into the run's volume, so the leg
always builds what it names. The leg runs detached: closing the terminal does not
stop it, and running this script again for the same run re-attaches instead of
starting a second one, so a watcher may be stopped (Ctrl+C, a tool timeout) and
resumed with -Follow at no cost to the leg. Run it in the foreground and stay with
it; -Stop if you must leave.

Output: build-docker\<short-sha>\, or build-docker\<short-sha>-<run>\ for a named
run (leg.log, build.log, build-errors.txt, ctest.log, ctest-junit.xml, tests.txt,
agreement.txt, summary.txt). Exit code: the leg's (0 pass, 1 fail).
#>
param(
    [string] $Commit = 'HEAD',
    [ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_.-]*$')] [string] $Run = '',
    [ValidateSet('all', 'image', 'sync', 'configure', 'build', 'test', 'consumer')] [string] $Step = 'all',
    [string] $Memory = '8g',
    [ValidateRange(1, 64)] [int] $Jobs = 8,
    [int] $MinFreeGB = 4,
    [switch] $NoSeed,
    [switch] $Follow,
    [switch] $Stop,
    [switch] $Clean,
    [switch] $Prune,
    [ValidateRange(0, 1000)] [int] $Keep = 4
)
$ErrorActionPreference = 'Stop'
$Prefix = 'spade-docker-leg'
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

# State of a leg's container: '' if there is none, else 'running', 'exited', ...
function Get-LegState([string] $Name) {
    (& docker ps -a --filter "name=^$Name`$" --format '{{.State}}') -join ''
}

function Get-LegLabels([string] $Name) {
    $raw = (& docker ps -a --filter "name=^$Name`$" --format '{{.Labels}}') -join ''
    $labels = @{}
    foreach ($pair in ($raw -split ',')) {
        $kv = $pair -split '=', 2
        if ($kv.Count -eq 2) { $labels[$kv[0]] = $kv[1] }
    }
    $labels
}

function Test-VolumeExists([string] $Volume) {
    @(& docker volume ls -q --filter "name=$Volume") -contains $Volume
}

# Every leg volume, newest first: the per-run ones (label spade.leg) and the
# single volume every leg shared before runs had their own (spade-docker-leg).
function Get-LegVolumes {
    $names = @(& docker volume ls -q --filter 'label=spade.leg')
    Invoke-Checked 'docker volume ls'
    if (Test-VolumeExists $Prefix) { $names += $Prefix }
    if ($names.Count -eq 0) { return @() }
    $rows = @(& docker volume inspect --format '{{.CreatedAt}} {{.Name}}' @names)
    Invoke-Checked 'docker volume inspect'
    @($rows | Sort-Object -Descending | ForEach-Object { ($_ -split ' ', 2)[1] })
}

# Whether a container mounts the volume: a running one, or with -AnyState any.
function Test-VolumeInUse([string] $Volume, [switch] $AnyState) {
    $psArgs = @('ps', '-q', '--filter', "volume=$Volume")
    if ($AnyState) { $psArgs += '-a' }
    [bool]((& docker @psArgs) -join '')
}

# Streams the running leg's log, waits for it, removes it, prints the summary.
function Watch-Leg([string] $Name) {
    $labels = Get-LegLabels $Name
    Write-Host ("docker-leg: following {0} on {1}, step {2}" -f $Name, $labels['spade.commit'], $labels['spade.step'])
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
    # How the volume began and what it held before this run, from the files
    # the driver's seed and sync left in it; for any commit, old or new.
    $vol = $labels['spade.volume']
    $img = $labels['spade.image']
    if ($summary -and (Test-Path $summary) -and $vol -and $img -and
        -not (Select-String -Path $summary -Pattern '^volume ' -Quiet)) {
        $info = @(& docker run --rm -v "${vol}:/leg:ro" $img sh -c `
            'cat /leg/seed 2>/dev/null || echo before seeding was recorded; cat /leg/previous 2>/dev/null || echo an unrecorded commit')
        if ($LASTEXITCODE -eq 0 -and $info.Count -ge 2) {
            $line = "volume      {0}: began {1}; held {2} before this run" -f $vol, $info[0], $info[1]
            Add-Content -Path $summary -Value $line
            Write-Host $line
        }
    }
    if ($summary -and (Test-Path $summary) -and -not (Select-String -Path $summary -Pattern '^host ' -Quiet)) {
        Add-Content -Path $summary `
            -Value ("host        {0} GB free on {1} after the run" -f $free, $env:LOCALAPPDATA.Substring(0, 2))
    }
    Write-Host ("docker-leg: exit {0}; {1} GB free on the host; output in {2}" -f $code, $free, $out)
    return $code
}

function Resolve-Commit {
    $sha = ((& git -C $Repo rev-parse --verify "$Commit^{commit}") -join '').Trim()
    Invoke-Checked "git rev-parse $Commit"
    $sha
}

# ---- prune: idle leg volumes beyond the newest $Keep ---------------------------
if ($Prune) {
    $kept = 0
    foreach ($v in Get-LegVolumes) {
        if (Test-VolumeInUse $v -AnyState) { Write-Host "docker-leg: keeping $v (a container uses it)"; continue }
        if ($kept -lt $Keep) { $kept++; Write-Host "docker-leg: keeping $v"; continue }
        & docker volume rm $v | Out-Null
        Invoke-Checked "docker volume rm $v"
        Write-Host "docker-leg: removed $v"
    }
    exit 0
}

# ---- which run ---------------------------------------------------------------
$sha = $null
if ($Run) {
    $id = $Run
} elseif (($Follow -or $Stop) -and -not $PSBoundParameters.ContainsKey('Commit')) {
    # No run named: the only leg there is.
    $legs = @(& docker ps -a --filter 'label=spade.leg' --format '{{.Names}}')
    if ($legs.Count -eq 0) { Write-Host 'docker-leg: no leg is running'; exit 0 }
    if ($legs.Count -gt 1) {
        Write-Host ("docker-leg: {0} legs exist ({1}); name one with -Run" -f $legs.Count, ($legs -join ', '))
        exit 2
    }
    $id = $legs[0].Substring($Prefix.Length + 1)
} else {
    $sha = Resolve-Commit
    $id = $sha.Substring(0, 12)
}
$Name = "$Prefix-$id"
$Volume = "$Prefix-$id"

# ---- this run's existing leg comes first -------------------------------------
$state = Get-LegState $Name
if ($Stop) {
    if ($state -eq '') { Write-Host "docker-leg: no leg to stop for run $id"; exit 0 }
    & docker stop $Name | Out-Null
    & docker rm $Name | Out-Null
    Write-Host "docker-leg: stopped $Name"
    exit 0
}
if ($state -eq 'running') {
    if (-not $Follow) {
        Write-Host "docker-leg: run $id is already running; following it instead of starting another (-Commit and -Step are ignored)"
    }
    exit (Watch-Leg $Name)
}
if ($state -ne '') {
    # A finished leg nobody waited for, e.g. after a closed terminal.
    Write-Host "docker-leg: found a finished leg ($state) for run $id that nobody collected"
    $code = Watch-Leg $Name
    if ($Follow) { exit $code }
} elseif ($Follow) {
    Write-Host "docker-leg: no leg is running for run $id"
    exit 0
}

if ($Clean) {
    if (Test-VolumeExists $Volume) {
        & docker volume rm $Volume | Out-Null
        Invoke-Checked 'docker volume rm'
    }
    Write-Host "docker-leg: volume $Volume removed"
    exit 0
}

# ---- the commit ------------------------------------------------------------------
if (-not $sha) { $sha = Resolve-Commit }
$dirty = @(& git -C $Repo status --porcelain --untracked-files=no).Count
if ($dirty -gt 0) {
    Write-Host "docker-leg: note: $dirty tracked file(s) have uncommitted changes; the leg tests $sha without them"
}

$free = Get-FreeGB
Write-Host ("docker-leg: {0} GB free on the host" -f $free)
if ($free -lt $MinFreeGB) {
    throw ("{0} GB free is under the {1} GB floor: a full disk truncates files mid-build (08-lessons). Free space, then run again" -f $free, $MinFreeGB)
}

$short = $sha.Substring(0, 12)
$outName = if ($id -eq $short) { $short } else { "$short-$id" }
$out = Join-Path $Repo ('build-docker\' + $outName)
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
# But the driver's own copy seeds and syncs: those steps arrange the volume,
# which is the driver's business, and an older commit's copy may predate them.
Copy-Item (Join-Path $PSScriptRoot 'docker-leg.sh') (Join-Path $out 'docker-leg-driver.sh') -Force
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

# ---- the run's volume: a new one starts as a copy of the newest idle one ------
if (-not (Test-VolumeExists $Volume)) {
    $seed = $null
    if (-not $NoSeed) {
        $seed = Get-LegVolumes | Where-Object { -not (Test-VolumeInUse $_) } | Select-Object -First 1
    }
    & docker volume create --label spade.leg=1 --label "spade.run=$id" $Volume | Out-Null
    Invoke-Checked 'docker volume create'
    $seedArgs = @()
    $seedMounts = @()
    if ($seed) {
        Write-Host "docker-leg: new volume $Volume, seeded from $seed"
        $seedArgs = @('--from', $seed)
        $seedMounts = @('-v', "${seed}:/seed:ro")
    } else {
        Write-Host "docker-leg: new volume $Volume, empty"
    }
    & docker run --rm --name "$Name-seed" @seedMounts -v "${Volume}:/leg" -v "${out}:/out" `
        $image bash /out/docker-leg-driver.sh seed @seedArgs
    if ($LASTEXITCODE -ne 0) {
        # A half-made volume would pass for a whole one next time.
        $code = $LASTEXITCODE
        & docker volume rm $Volume | Out-Null
        throw "seeding $Volume failed (exit $code); the volume is removed"
    }
}

# ---- sync, then the step ---------------------------------------------------------
$mounts = @('-v', "${Volume}:/leg", '-v', "${out}:/out")
& docker run --rm --name "$Name-sync" @mounts $image bash /out/docker-leg-driver.sh sync
Invoke-Checked 'sync'
if ($Step -eq 'sync') { exit 0 }

Write-Host "docker-leg: starting $Name, step $Step on $sha (memory $Memory, jobs $Jobs)"
& docker run --detach --name $Name --memory $Memory --memory-swap $Memory `
    --label spade.leg=1 --label "spade.run=$id" `
    --label "spade.commit=$sha" --label "spade.step=$Step" --label "spade.out=$out" `
    --label "spade.volume=$Volume" --label "spade.image=$image" `
    @mounts $image bash /out/docker-leg.sh $Step --jobs $Jobs | Out-Null
Invoke-Checked 'docker run'
exit (Watch-Leg $Name)
