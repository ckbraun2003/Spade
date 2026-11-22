<#
.SYNOPSIS
  Starts and stops ONE PX4 SITL container for a run (O140, O144; docs/design/interface/plans/2026-11-21-fc-links.md section 4).

.DESCRIPTION
  PX4 SITL runs in the image spade-px4-sitl:v1.17.0 (Gazebo-free board variant, network off at build; tools/fc_link/px4/Dockerfile and
  Dockerfile.airframes). PX4 CONNECTS to the simulator, which is Spade: the container is told where to dial with PX4_SIM_HOSTNAME, and
  Spade listens on TCP 4560 on the host. Nothing here touches Docker Desktop itself; it only creates and removes the container named
  spade-px4-<RunId>.

    (no -Stop)      exit 0 and the line "px4-sitl: up <name> -> <host>:4560" when the container runs;
                    exit 2 and one reason line when the image is missing ("px4-sitl: no autopilot image ...");
                    exit 3 when docker is not reachable; exit 4 when the container did not stay up.
    -Stop           stops the container (SIGTERM, so PX4 closes its log), copies its logs into the -OutDir given at start (kept as a container
                    label; -OutDir may also be passed again), removes it, and prints the path of the newest .ulg as the LAST line (a bare
                    path, or "none" when there was no log); exit 0. px4-stdout.log is TIMESTAMP-ORDERED (docker logs --timestamps, stdout and stderr
                    merged by a stable sort on the time prefix; the prefix stays in the file). -Stop never reports success when it failed: every step is
                    checked, a failure is named on a line `px4-sitl: -Stop FAILED: ...` (docker stop, the run folder, px4-stdout.log missing or empty,
                    docker cp, docker rm, the container still listed) and the exit is 7; the container is removed whatever else failed. Exit 6: no run
                    folder known. The LAST line is always the .ulg path or `none`.

  The run id is the caller's (Test/Docs' fc-run.ps1 passes the run folder's id). One container per run: starting a second with the same
  id is refused (exit 5).
#>
param(
    [Parameter(Mandatory = $true)][ValidatePattern('^[A-Za-z0-9][A-Za-z0-9_.-]*$')][string]$RunId,
    [string]$HostAddress = 'host.docker.internal',   # what PX4 connects to: the machine Spade listens on
    [string]$OutDir,                                  # the run folder: the PX4 log and its stdout land here on -Stop
    [switch]$Stop,                                    # stop and collect instead of starting
    [string]$Image = 'spade-px4-sitl:v1.17.0',
    [int]$Autostart = 10090,                          # the airframe: 10090 = PX4's own x500 numbers, 10091 = the kit (when it exists)
    [int]$SimPort = 4560,                             # PX4's simulator port; the container's instance 0
    [switch]$PublishOffboard,                         # publish PX4's offboard API port (UDP 14540) on this machine's loopback only, for a companion computer
    [string]$Network = '',                            # docker's --network for the container; empty = docker's default bridge, which a real run needs (PX4 must reach the host's TCP 4560). A check with no route passes `none`: the --add-host entry still resolves
    [string]$Docker = 'docker'                        # the docker command; a test points it at a fake
)

# 'Continue', not 'Stop' (as fc-run.ps1): Windows PowerShell 5.1 turns a native command's stderr under 2>&1 into a terminating error, and docker and PX4 write there.
# Every docker call below is checked by its own exit code.
$ErrorActionPreference = 'Continue'
$name = "spade-px4-$RunId"
$px4Dir = '/src/PX4-Autopilot/build/px4_sitl_spade'

& $Docker version --format '{{.Server.Version}}' *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Output 'px4-sitl: docker is not reachable (is Docker Desktop running?)'
    exit 3
}

if (-not $Stop) {
    & $Docker image inspect $Image *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Output "px4-sitl: no autopilot image $Image (build it from tools/fc_link/px4: Dockerfile then Dockerfile.airframes, network off)"
        exit 2
    }
    $existing = (& $Docker ps -a --filter "name=^/$name$" --format '{{.Names}}')
    if ($existing) {
        Write-Output "px4-sitl: a container named $name already exists (one container per run id)"
        exit 5
    }
    if ($OutDir) { $OutDir = [System.IO.Path]::GetFullPath($OutDir) }
    $dockerArgs = @('run', '-d', '--name', $name, '--label', "spade.outdir=$OutDir",
        '--add-host', 'host.docker.internal:host-gateway',
        '-e', "PX4_SYS_AUTOSTART=$Autostart",
        '-e', "PX4_SIM_HOSTNAME=$HostAddress",
        '-w', "$px4Dir/rootfs",
        '--entrypoint', 'sh')
    if ($Network) { $dockerArgs += @('--network', $Network) }
    if ($PublishOffboard) { $dockerArgs += @('-p', '127.0.0.1:14540:14540/udp') }
    # PX4's log folder is created first, so -Stop's docker cp always finds one (empty = no log = `none`); then sh gives way to px4 (exec).
    $dockerArgs += @($Image, '-c', "mkdir -p $px4Dir/rootfs/log && exec $px4Dir/bin/px4 $px4Dir/etc -s etc/init.d-posix/rcS -d")
    & $Docker @dockerArgs *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Output "px4-sitl: docker could not start $name from $Image"
        exit 4
    }
    Start-Sleep -Seconds 2
    $running = (& $Docker ps --filter "name=^/$name$" --format '{{.Names}}')
    if (-not $running) {
        Write-Output "px4-sitl: $name exited at start; its log follows"
        & $Docker logs $name
        & $Docker rm -f $name *> $null
        exit 4
    }
    Write-Output "px4-sitl: up $name -> ${HostAddress}:$SimPort"
    exit 0
}

# stop
$existing = (& $Docker ps -a --filter "name=^/$name$" --format '{{.Names}}')
if (-not $existing) {
    Write-Output "px4-sitl: no container named $name"
    Write-Output 'none'
    exit 0
}
if (-not $OutDir) { $OutDir = (& $Docker inspect --format '{{ index .Config.Labels "spade.outdir" }}' $name) }
if (-not $OutDir) {
    Write-Output 'px4-sitl: no run folder known (pass -OutDir)'
    & $Docker rm -f $name *> $null
    Write-Output 'none'
    exit 6
}

# From here on every step is checked and a failure is NAMED; the container is still removed whatever else failed, and the exit is 7 if anything did.
$failures = New-Object System.Collections.Generic.List[string]
$folderOk = $true
try { New-Item -ItemType Directory -Force -Path $OutDir -ErrorAction Stop | Out-Null }
catch { $folderOk = $false; $failures.Add("the run folder cannot be created ($OutDir)") | Out-Null }
if ($folderOk -and -not (Test-Path -LiteralPath $OutDir -PathType Container)) { $folderOk = $false; $failures.Add("the run folder cannot be created ($OutDir)") | Out-Null }  # New-Item -Force can stay silent under a file

& $Docker stop -t 15 $name *> $null
if ($LASTEXITCODE -ne 0) { $failures.Add("docker stop failed for $name (exit $LASTEXITCODE)") | Out-Null }

if ($folderOk) {
    # docker prints a container's stdout and its stderr as two separate blocks; --timestamps puts each line's time in front of it, and a STABLE sort on that
    # prefix (ties keep docker's order; a line without one stays with the line before it) restores the order the lines were written in.
    $lines = @(& $Docker logs --timestamps $name 2>&1 | ForEach-Object { "$_" })
    $rows = New-Object System.Collections.Generic.List[object]
    $key = ''
    $n = 0
    foreach ($l in $lines) {
        if ($l -match '^(\d{4}-\d\d-\d\dT[0-9:.]+Z)\s') { $key = $Matches[1] }
        $rows.Add([pscustomobject]@{ K = $key; I = $n; L = $l }) | Out-Null
        $n++
    }
    $logFile = Join-Path $OutDir 'px4-stdout.log'
    try { @($rows | Sort-Object K, I | ForEach-Object { $_.L }) | Set-Content -Path $logFile -Encoding UTF8 -ErrorAction Stop }
    catch { $failures.Add('px4-stdout.log could not be written') | Out-Null }
    $f = Get-Item -LiteralPath $logFile -ErrorAction SilentlyContinue
    if (-not $f -or $f.Length -le 3) { $failures.Add('px4-stdout.log is missing or empty (docker logs gave nothing)') | Out-Null }  # 3 bytes = the UTF-8 mark alone

    $logDir = Join-Path $OutDir 'px4-log'
    & $Docker cp "${name}:$px4Dir/rootfs/log" $logDir *> $null
    if ($LASTEXITCODE -ne 0) { $failures.Add("docker cp of the PX4 log folder failed (exit $LASTEXITCODE)") | Out-Null }
}

& $Docker rm -f $name *> $null
if ($LASTEXITCODE -ne 0) { $failures.Add("docker rm -f $name failed (exit $LASTEXITCODE)") | Out-Null }
$left = (& $Docker ps -a --filter "name=^/$name$" --format '{{.Names}}')
if ($left) { $failures.Add("container $name still exists after docker rm") | Out-Null }

$ulg = $null
if ($folderOk) { $ulg = Get-ChildItem -Path (Join-Path $OutDir 'px4-log') -Recurse -Filter '*.ulg' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime | Select-Object -Last 1 }
foreach ($f in $failures) { Write-Output "px4-sitl: -Stop FAILED: $f" }
if ($ulg) { Write-Output $ulg.FullName } else { Write-Output 'none' }
if ($failures.Count -gt 0) { exit 7 }
exit 0
