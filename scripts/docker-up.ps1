<#
.SYNOPSIS
    Make sure the Docker daemon answers: start Docker Desktop if it does not, and wait for it. scripts\docker-leg.ps1 and scripts\gcc-check.sh call it first.

.DESCRIPTION
    Kat quits Docker Desktop whenever no Spade Linux work is running or queued (it frees about 6.6 GB for the shared queue), so a leg or a gcc check
    often finds the daemon down. `docker info` answers: nothing to do, exit 0 at once. It fails: this starts Docker Desktop and polls `docker info`
    every PollS seconds, one log line per poll, until it answers or LimitS seconds have passed. A cold start takes about 21 s on a quiet box (24 s wall); once over 180 s with builds and legs queued (2026-11-19).

    It never stops Docker Desktop: Kat quits it when idle.

    Where Docker Desktop is: SPADE_DOCKER_DESKTOP when set (the only place looked at; a test uses it), else, in this order, under LOCALAPPDATA
    (Programs\DockerDesktop\frontend, Programs\Docker\Docker) and under ProgramFiles and ProgramFiles(x86) (Docker\Docker). No user name is written down.

    Exit: 0 the daemon answers (it was up, or it came up); 1 it never did, or Docker Desktop was not found; the last line says why.

.PARAMETER LimitS
    Seconds to wait for the daemon after starting Docker Desktop. Default 180 (SPADE_DOCKER_START_LIMIT_S sets another default).

.PARAMETER PollS
    Seconds between polls. Default 5 (SPADE_DOCKER_POLL_S sets another default).

.EXAMPLE
    powershell -NoProfile -ExecutionPolicy Bypass -File scripts\docker-up.ps1
#>
param(
    [int] $LimitS = $(if ($env:SPADE_DOCKER_START_LIMIT_S) { [int] $env:SPADE_DOCKER_START_LIMIT_S } else { 180 }),
    [int] $PollS = $(if ($env:SPADE_DOCKER_POLL_S) { [int] $env:SPADE_DOCKER_POLL_S } else { 5 })
)
$ErrorActionPreference = 'Stop'

function Test-DockerDaemon {
    # `docker info` exits non-zero with the daemon down, and prints its error to stderr, which Windows PowerShell 5.1 turns into a
    # terminating error under 'Stop': only the exit code counts.
    $saved = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try { & docker info *> $null } finally { $ErrorActionPreference = $saved }
    $LASTEXITCODE -eq 0
}

function Find-DockerDesktop {
    # The places Docker Desktop installs to, or the one SPADE_DOCKER_DESKTOP names; returns the first that exists, else $null.
    if ($env:SPADE_DOCKER_DESKTOP) {
        if (Test-Path -LiteralPath $env:SPADE_DOCKER_DESKTOP -PathType Leaf) { return $env:SPADE_DOCKER_DESKTOP }
        return $null
    }
    $candidates = @()
    if ($env:LOCALAPPDATA) {
        $candidates += Join-Path $env:LOCALAPPDATA 'Programs\DockerDesktop\frontend\Docker Desktop.exe'
        $candidates += Join-Path $env:LOCALAPPDATA 'Programs\Docker\Docker\Docker Desktop.exe'
    }
    foreach ($pf in @($env:ProgramFiles, ${env:ProgramFiles(x86)})) {
        if ($pf) { $candidates += Join-Path $pf 'Docker\Docker\Docker Desktop.exe' }
    }
    foreach ($c in $candidates) { if (Test-Path -LiteralPath $c -PathType Leaf) { return $c } }
    $null
}

function Start-DockerDaemon([int] $LimitS, [int] $PollS) {
    # Returns $true when the daemon answers, after starting Docker Desktop if it had to. Writes one line per poll.
    if (-not (Get-Command docker -ErrorAction SilentlyContinue)) { Write-Host 'docker-up: docker is not on PATH (Docker Desktop is not installed, or this shell predates it)'; return $false }
    if (Test-DockerDaemon) { return $true }
    $exe = Find-DockerDesktop
    if (-not $exe) {
        $where = if ($env:SPADE_DOCKER_DESKTOP) { $env:SPADE_DOCKER_DESKTOP } else { 'under LOCALAPPDATA\Programs and ProgramFiles' }
        Write-Host "docker-up: Docker Desktop is not installed where it was looked for ($where); start the Docker daemon by hand"
        return $false
    }
    Write-Host "docker-up: the Docker daemon is down; starting Docker Desktop ($exe), waiting up to $LimitS s"
    $clock = [Diagnostics.Stopwatch]::StartNew()
    Start-Process -FilePath $exe -WindowStyle Hidden | Out-Null
    while ($clock.Elapsed.TotalSeconds -lt $LimitS) {
        Start-Sleep -Seconds $PollS
        if (Test-DockerDaemon) {
            Write-Host ("docker-up: the Docker daemon answered after {0:N0} s" -f $clock.Elapsed.TotalSeconds)
            return $true
        }
        Write-Host ("docker-up: waiting for the Docker daemon ({0:N0} s of {1})" -f $clock.Elapsed.TotalSeconds, $LimitS)
    }
    Write-Host "docker-up: the Docker daemon did not answer within $LimitS s of starting Docker Desktop"
    $false
}

if (Start-DockerDaemon $LimitS $PollS) { exit 0 }
exit 1
