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
                    path, or "none" when there was no log); exit 0.

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
    [switch]$PublishOffboard                          # publish PX4's offboard API port (UDP 14540) on this machine's loopback only, for a companion computer
)

# 'Continue', not 'Stop' (as fc-run.ps1): Windows PowerShell 5.1 turns a native command's stderr under 2>&1 into a terminating error, and docker and PX4 write there.
# Every docker call below is checked by its own exit code.
$ErrorActionPreference = 'Continue'
$name = "spade-px4-$RunId"
$px4Dir = '/src/PX4-Autopilot/build/px4_sitl_spade'

& docker version --format '{{.Server.Version}}' *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Output 'px4-sitl: docker is not reachable (is Docker Desktop running?)'
    exit 3
}

if (-not $Stop) {
    & docker image inspect $Image *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Output "px4-sitl: no autopilot image $Image (build it from tools/fc_link/px4: Dockerfile then Dockerfile.airframes, network off)"
        exit 2
    }
    $existing = (& docker ps -a --filter "name=^/$name$" --format '{{.Names}}')
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
        '--entrypoint', "$px4Dir/bin/px4")
    if ($PublishOffboard) { $dockerArgs += @('-p', '127.0.0.1:14540:14540/udp') }
    $dockerArgs += @($Image, "$px4Dir/etc", '-s', 'etc/init.d-posix/rcS', '-d')
    & docker @dockerArgs *> $null
    if ($LASTEXITCODE -ne 0) {
        Write-Output "px4-sitl: docker could not start $name from $Image"
        exit 4
    }
    Start-Sleep -Seconds 2
    $running = (& docker ps --filter "name=^/$name$" --format '{{.Names}}')
    if (-not $running) {
        Write-Output "px4-sitl: $name exited at start; its log follows"
        & docker logs $name
        & docker rm -f $name *> $null
        exit 4
    }
    Write-Output "px4-sitl: up $name -> ${HostAddress}:$SimPort"
    exit 0
}

# stop
$existing = (& docker ps -a --filter "name=^/$name$" --format '{{.Names}}')
if (-not $existing) {
    Write-Output "px4-sitl: no container named $name"
    Write-Output 'none'
    exit 0
}
if (-not $OutDir) { $OutDir = (& docker inspect --format '{{ index .Config.Labels "spade.outdir" }}' $name) }
if (-not $OutDir) {
    Write-Output 'px4-sitl: no run folder known (pass -OutDir)'
    & docker rm -f $name *> $null
    Write-Output 'none'
    exit 6
}
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
& docker stop -t 15 $name *> $null
& docker logs $name 2>&1 | ForEach-Object { "$_" } | Set-Content -Path (Join-Path $OutDir 'px4-stdout.log') -Encoding UTF8  # UTF-8, not the UTF-16 a `>` writes
$logDir = Join-Path $OutDir 'px4-log'
& docker cp "${name}:$px4Dir/rootfs/log" $logDir *> $null
& docker rm -f $name *> $null
$ulg = Get-ChildItem -Path $logDir -Recurse -Filter '*.ulg' -ErrorAction SilentlyContinue | Sort-Object LastWriteTime | Select-Object -Last 1
if ($ulg) { Write-Output $ulg.FullName } else { Write-Output 'none' }
exit 0
