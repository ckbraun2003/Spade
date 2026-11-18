<#
.SYNOPSIS
    A down Docker daemon gets started (scripts\docker-up.ps1, called first by scripts\docker-leg.ps1 and scripts\gcc-check.sh), with a fake `docker` and a fake
    Docker Desktop on PATH: no real Docker is touched, so it runs anywhere and never starts or stops the real one.

.DESCRIPTION
    The fake `docker info` fails until a marker file exists; the fake Docker Desktop writes a line to a "started" file and, when the case says so, creates
    the marker two seconds later. Every `docker` call is logged to a calls file. Cases:
      up         the daemon answers: exit 0, Docker Desktop is not started
      starts     down, then it comes up: exit 0, Docker Desktop started once, one log line per poll, the time it took
      never      it never comes up: exit 1 after the limit, the reason named
      missing    Docker Desktop is not where it is looked for: exit 1, nothing started
      no stop    no docker call in any case but `info` (it never stops anything)
      leg        scripts\docker-leg.ps1 -Stop on a down daemon starts it first, then goes on (no leg to stop: exit 0)
      gcc-check  scripts\gcc-check.sh on a down daemon starts it first (the fake has no image, so it then exits 2 naming that)

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tests\docker_up_test.ps1 <repo root> [<bash.exe>]
#>
param([Parameter(Mandatory)] [string] $RepoRoot, [string] $Bash = '')
# 'Continue', not 'Stop': Windows PowerShell 5.1 turns a native command's stderr under 2>&1 into a terminating error, and the scripts under test write there.
$ErrorActionPreference = 'Continue'
$problems = New-Object System.Collections.Generic.List[string]
function Expect([string] $Case, [bool] $Ok, [string] $What) { if (-not $Ok) { $problems.Add("${Case}: $What") | Out-Null } }

$up = Join-Path $RepoRoot 'scripts\docker-up.ps1'
$leg = Join-Path $RepoRoot 'scripts\docker-leg.ps1'
$gcc = Join-Path $RepoRoot 'scripts\gcc-check.sh'
$work = Join-Path ([IO.Path]::GetTempPath()) ('spade-docker-up-test-' + [guid]::NewGuid().ToString('N').Substring(0, 12))
New-Item -ItemType Directory -Force -Path $work | Out-Null
$fwd = { param($p) $p.Replace('\', '/') }
$ps = Join-Path $PSHOME 'powershell.exe'

try {
    $fake = Join-Path $work 'fake'
    New-Item -ItemType Directory -Force -Path $fake | Out-Null
    $marker = Join-Path $work 'daemon-up'
    $started = Join-Path $work 'started.txt'
    $calls = Join-Path $work 'calls.txt'
    # `docker` for PowerShell (docker.cmd) and for Git Bash (a script without an extension).
    [IO.File]::WriteAllText((Join-Path $fake 'docker.cmd'), @"
@echo off
echo %*>>"%FAKE_DOCKER_CALLS%"
if /i "%~1"=="info" (
  if exist "%FAKE_DOCKER_UP%" exit /b 0
  echo Cannot connect to the Docker daemon 1>&2
  exit /b 1
)
exit /b 0
"@.Replace("`r`n", "`n").Replace("`n", "`r`n"))
    [IO.File]::WriteAllText((Join-Path $fake 'docker'), "#!/bin/sh`necho `"`$*`" >> `"`$FAKE_DOCKER_CALLS`"`nif [ `"`$1`" = info ]; then [ -e `"`$FAKE_DOCKER_UP`" ] && exit 0; echo 'Cannot connect to the Docker daemon' >&2; exit 1; fi`nexit 0`n")
    [IO.File]::WriteAllText((Join-Path $fake 'Docker Desktop.cmd'), @"
@echo off
echo started>>"%FAKE_DOCKER_STARTED%"
if defined FAKE_DOCKER_COMES_UP (
  ping -n 3 127.0.0.1 >nul
  echo up>"%FAKE_DOCKER_UP%"
)
"@.Replace("`r`n", "`n").Replace("`n", "`r`n"))

    $env:PATH = "$fake;$env:PATH"
    $env:FAKE_DOCKER_UP = & $fwd $marker
    $env:FAKE_DOCKER_STARTED = & $fwd $started
    $env:FAKE_DOCKER_CALLS = & $fwd $calls
    $env:SPADE_DOCKER_DESKTOP = Join-Path $fake 'Docker Desktop.cmd'
    $env:SPADE_DOCKER_POLL_S = '1'

    function Reset([bool] $DaemonUp, [bool] $ComesUp) {
        Remove-Item -LiteralPath $marker, $started, $calls -ErrorAction SilentlyContinue
        if ($DaemonUp) { Set-Content -LiteralPath $marker 'up' }
        if ($ComesUp) { $env:FAKE_DOCKER_COMES_UP = '1' } else { Remove-Item Env:FAKE_DOCKER_COMES_UP -ErrorAction SilentlyContinue }
    }
    function Started { if (Test-Path -LiteralPath $started) { @(Get-Content -LiteralPath $started).Count } else { 0 } }
    function Calls { if (Test-Path -LiteralPath $calls) { @(Get-Content -LiteralPath $calls) } else { @() } }
    $allCalls = New-Object System.Collections.Generic.List[string]

    # up
    Reset $true $false
    $out = (& $ps -NoProfile -ExecutionPolicy Bypass -File $up -LimitS 10 2>&1 | Out-String)
    Expect 'up' ($LASTEXITCODE -eq 0) "exit $LASTEXITCODE"
    Expect 'up' ((Started) -eq 0) 'Docker Desktop was started with the daemon up'
    Expect 'up' (-not $out.Contains('starting Docker Desktop')) 'it announced a start'
    $allCalls.AddRange([string[]](Calls))

    # starts
    Reset $false $true
    $clock = [Diagnostics.Stopwatch]::StartNew()
    $out = (& $ps -NoProfile -ExecutionPolicy Bypass -File $up -LimitS 60 2>&1 | Out-String)
    $took = $clock.Elapsed.TotalSeconds
    Expect 'starts' ($LASTEXITCODE -eq 0) "exit ${LASTEXITCODE}: $out"
    Expect 'starts' ((Started) -eq 1) "Docker Desktop started $(Started) times, expected once"
    Expect 'starts' ($out.Contains('the Docker daemon is down; starting Docker Desktop')) 'no start line'
    Expect 'starts' ($out -match 'waiting for the Docker daemon \(\d+ s of 60\)') 'no wait line'
    Expect 'starts' ($out -match 'the Docker daemon answered after \d+ s') 'no answered line'
    Expect 'starts' ($took -lt 30) "took $took s with a daemon that comes up in 2 s"
    $allCalls.AddRange([string[]](Calls))

    # never
    Reset $false $false
    $out = (& $ps -NoProfile -ExecutionPolicy Bypass -File $up -LimitS 3 2>&1 | Out-String)
    Expect 'never' ($LASTEXITCODE -eq 1) "exit $LASTEXITCODE"
    Expect 'never' ((Started) -eq 1) 'Docker Desktop was not started once'
    Expect 'never' ($out.Contains('did not answer within 3 s')) "the reason is not named: $out"
    $allCalls.AddRange([string[]](Calls))

    # missing
    Reset $false $true
    $saved = $env:SPADE_DOCKER_DESKTOP
    $env:SPADE_DOCKER_DESKTOP = Join-Path $fake 'no such Docker Desktop.exe'
    $out = (& $ps -NoProfile -ExecutionPolicy Bypass -File $up -LimitS 3 2>&1 | Out-String)
    $env:SPADE_DOCKER_DESKTOP = $saved
    Expect 'missing' ($LASTEXITCODE -eq 1) "exit $LASTEXITCODE"
    Expect 'missing' ((Started) -eq 0) 'something was started'
    Expect 'missing' ($out.Contains('is not installed where it was looked for')) "the reason is not named: $out"
    $allCalls.AddRange([string[]](Calls))

    # leg: docker-leg.ps1 -Stop with the daemon down: started first, then "no leg to stop" (the fake docker answers every call with nothing)
    if (Test-Path -LiteralPath $leg) {
        Reset $false $true
        $out = (& $ps -NoProfile -ExecutionPolicy Bypass -File $leg -Stop -Run docker-up-test 2>&1 | Out-String)
        Expect 'leg' ($LASTEXITCODE -eq 0) "exit ${LASTEXITCODE}: $out"
        Expect 'leg' ((Started) -eq 1) 'docker-leg.ps1 did not start Docker Desktop'
        Expect 'leg' ($out.Contains('no leg to stop for run docker-up-test')) "it did not go on after the start: $out"
        $allCalls.AddRange([string[]](Calls))
    }

    # gcc-check: the same from Git Bash. Run from a folder with engine/ and tests/ and one file; the fake docker has no image, so exit 2 after the start.
    if ($Bash -and (Test-Path -LiteralPath $Bash) -and (Test-Path -LiteralPath $gcc)) {
        $tree = Join-Path $work 'tree'
        New-Item -ItemType Directory -Force -Path (Join-Path $tree 'engine'), (Join-Path $tree 'tests') | Out-Null
        Set-Content -LiteralPath (Join-Path $tree 'tests\t.cpp') 'int main() { return 0; }'
        Reset $false $true
        Push-Location $tree
        try { $out = ((& $Bash (& $fwd $gcc) 'tests/t.cpp' 2>&1 | ForEach-Object { "$_" }) -join "`n"); $code = $LASTEXITCODE } finally { Pop-Location }
        Expect 'gcc-check' ((Started) -eq 1) "gcc-check.sh did not start Docker Desktop: $out"
        Expect 'gcc-check' ($out.Contains('starting Docker Desktop')) "no start line: $out"
        Expect 'gcc-check' ($code -eq 2 -and $out.Contains('no spade-docker-leg image')) "exit $code, expected 2 'no spade-docker-leg image' after the start: $out"
        $allCalls.AddRange([string[]](Calls))
    }

    # no stop: nothing but `info` (and the leg's own calls) was ever asked of docker; in particular no `stop`, `kill`, `desktop` or `quit`.
    foreach ($c in $allCalls) {
        Expect 'no stop' ($c -notmatch '^(stop|kill|desktop|quit|rm|system)\b') "a docker call that stops or removes: $c"
    }
    Write-Host ("docker_up_test: the fake daemon came up in {0:N1} s of wall time through docker-up.ps1 (it comes up 2 s after the start)" -f $took)
} finally {
    Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue
}

if ($problems.Count -eq 0) { Write-Host 'docker_up_test: all cases hold'; exit 0 }
foreach ($p in $problems) { Write-Host "docker_up_test: FAIL $p" }
exit 1
