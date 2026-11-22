<#
.SYNOPSIS
    scripts\px4-sitl.ps1 -Stop must not report success when it failed (Oversight, O140/O144). Runs it against a FAKE docker (a script handed to -Docker): no Docker,
    no PX4, runs anywhere.

.DESCRIPTION
    Cases (each runs `px4-sitl.ps1 -RunId t1 -Stop -OutDir <run folder> -Docker <fake>` and reads its exit code and output):
      ok          everything works: exit 0; the LAST line is the bare .ulg path; px4-stdout.log is timestamp-ordered (docker prints stdout and stderr as two
                  blocks, so the script sorts the lines by their timestamp prefix, stably); the container is gone; docker logs was asked for --timestamps
      rm fails    docker rm -f exits 1: exit 7, the text names the container and `docker rm`; the last line is still the .ulg path
      rm no-op    docker rm -f exits 0 but the container is still listed: exit 7, "still exists"
      empty log   docker logs prints nothing: exit 7, px4-stdout.log named
      stop fails  docker stop exits 1: exit 7, `docker stop` named
      cp fails    docker cp exits 1: exit 7, `docker cp` named
      bad folder  the run folder cannot be created: exit 7, the run folder named
      empty dir   docker cp finds an EMPTY log folder (PX4 died in boot, wrote nothing): exit 0 and the last line `none` (not a failure)
      start       a START creates PX4's log folder in the container first (so docker cp always finds one): the run command carries `mkdir -p <rootfs>/log`
                  and still ends in `exec <px4> <etc> -s etc/init.d-posix/rcS -d`; the script exits 0
      no box      no such container: exit 0 and `none` (as before)

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tests\px4_sitl_test.ps1 <repo root>
#>
param([Parameter(Mandatory)] [string] $RepoRoot)
$ErrorActionPreference = 'Continue'
$problems = New-Object System.Collections.Generic.List[string]
function Expect([string] $Case, [object] $Ok, [string] $What) {
    $good = if ($Ok -is [array]) { $Ok.Count -gt 0 } else { [bool] $Ok }
    if (-not $good) { $problems.Add("${Case}: $What") | Out-Null }
}

$script = Join-Path $RepoRoot 'scripts\px4-sitl.ps1'
$ps = Join-Path $PSHOME 'powershell.exe'
$work = Join-Path ([IO.Path]::GetTempPath()) ('spade-px4-sitl-test-' + [guid]::NewGuid().ToString('N').Substring(0, 12))
New-Item -ItemType Directory -Force -Path $work | Out-Null

try {
    $fake = Join-Path $work 'fake-docker.ps1'
    $state = Join-Path $work 'container.exists'
    $calls = Join-Path $work 'calls.txt'
    $logs = Join-Path $work 'logs.txt'
    # docker prints a container's stdout lines first, then its stderr lines; the timestamps are what restore the order.
    Set-Content -LiteralPath $logs @(
        '2026-11-21T10:00:01.000000001Z INFO  out-one',
        '2026-11-21T10:00:03.000000001Z INFO  out-three',
        '2026-11-21T10:00:02.000000001Z ERROR err-two',
        '2026-11-21T10:00:04.000000001Z ERROR err-four')
    Set-Content -LiteralPath (Join-Path $work 'fake-docker.ps1') @'
Add-Content -LiteralPath $env:FAKE_CALLS ($args -join ' ')
switch ($args[0]) {
    'version' { exit 0 }
    'image'   { exit 0 }
    'run'     { Set-Content -LiteralPath $env:FAKE_STATE 'x'; exit 0 }
    'ps'      { if (Test-Path $env:FAKE_STATE) { 'spade-px4-t1' }; exit 0 }
    'inspect' { $env:FAKE_LABELS_JSON; exit 0 }
    'stop'    { exit ([int] $env:FAKE_STOP_CODE) }
    'logs'    { if (-not $env:FAKE_LOGS_EMPTY) { Get-Content -LiteralPath $env:FAKE_LOGS_FILE }; exit 0 }
    'cp'      {
        if ([int] $env:FAKE_CP_CODE -ne 0) { exit ([int] $env:FAKE_CP_CODE) }
        $dst = $args[2]
        New-Item -ItemType Directory -Force -Path (Join-Path $dst 'sub') | Out-Null
        if (-not $env:FAKE_CP_EMPTY) { Set-Content -LiteralPath (Join-Path $dst 'sub\fake.ulg') 'ulg' }
        exit 0
    }
    'rm'      {
        if ([int] $env:FAKE_RM_CODE -ne 0) { exit ([int] $env:FAKE_RM_CODE) }
        if (-not $env:FAKE_RM_NOOP) { Remove-Item -LiteralPath $env:FAKE_STATE -ErrorAction SilentlyContinue }
        exit 0
    }
}
exit 0
'@

    function Run-Case([string] $Name, [hashtable] $Env, [string] $OutDir, [switch] $NoContainer) {
        foreach ($k in 'FAKE_STOP_CODE', 'FAKE_CP_CODE', 'FAKE_RM_CODE', 'FAKE_RM_NOOP', 'FAKE_LOGS_EMPTY', 'FAKE_CP_EMPTY') { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
        $env:FAKE_CALLS = $calls; $env:FAKE_STATE = $state; $env:FAKE_LOGS_FILE = $logs; $env:FAKE_LABELS_JSON = '{}'
        foreach ($k in $Env.Keys) { Set-Item "Env:$k" $Env[$k] }
        Remove-Item -LiteralPath $calls -ErrorAction SilentlyContinue
        if ($NoContainer) { Remove-Item -LiteralPath $state -ErrorAction SilentlyContinue } else { Set-Content -LiteralPath $state 'x' }
        if (Test-Path -LiteralPath $OutDir -PathType Container) { Remove-Item -Recurse -Force -LiteralPath $OutDir }
        $o = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -Stop -OutDir $OutDir -Docker $fake 2>&1 | ForEach-Object { "$_" }
        $code = $LASTEXITCODE
        return [pscustomobject]@{ Code = $code; Lines = @($o); Text = ($o -join "`n"); Calls = $(if (Test-Path -LiteralPath $calls) { Get-Content -LiteralPath $calls } else { @() }) }
    }

    # ok
    $out = Join-Path $work 'run-ok'
    $r = Run-Case 'ok' @{} $out
    Expect 'ok' ($r.Code -eq 0) "exit 0 (got $($r.Code)): $($r.Text)"
    Expect 'ok' ($r.Lines.Count -gt 0 -and $r.Lines[-1] -like '*fake.ulg') "the last line is the .ulg path (got '$($r.Lines[-1])')"
    $log = Join-Path $out 'px4-stdout.log'
    $got = if (Test-Path -LiteralPath $log) { Get-Content -LiteralPath $log } else { @() }
    $order = @($got | ForEach-Object { if ($_ -match 'out-one|err-two|out-three|err-four') { $Matches[0] } })
    Expect 'ok' (($order -join ',') -eq 'out-one,err-two,out-three,err-four') "px4-stdout.log is timestamp-ordered (got '$($order -join ',')')"
    Expect 'ok' (-not (Test-Path -LiteralPath $state)) 'the container is gone'
    Expect 'ok' (@($r.Calls | Where-Object { $_ -match '^logs .*--timestamps' }).Count -gt 0) 'docker logs was asked for --timestamps'

    # rm fails
    $r = Run-Case 'rm fails' @{ FAKE_RM_CODE = '1' } (Join-Path $work 'run-rm')
    Expect 'rm fails' ($r.Code -eq 7) "exit 7 (got $($r.Code)): $($r.Text)"
    Expect 'rm fails' ($r.Text -match 'docker rm') 'the text names docker rm'
    Expect 'rm fails' ($r.Text -match 'spade-px4-t1') 'the text names the container'
    Expect 'rm fails' ($r.Lines.Count -gt 0 -and $r.Lines[-1] -like '*fake.ulg') 'the last line is still the .ulg path'

    # rm no-op
    $r = Run-Case 'rm no-op' @{ FAKE_RM_NOOP = '1' } (Join-Path $work 'run-noop')
    Expect 'rm no-op' ($r.Code -eq 7) "exit 7 (got $($r.Code)): $($r.Text)"
    Expect 'rm no-op' ($r.Text -match 'still exists') 'the text says the container still exists'

    # empty log
    $r = Run-Case 'empty log' @{ FAKE_LOGS_EMPTY = '1' } (Join-Path $work 'run-empty')
    Expect 'empty log' ($r.Code -eq 7) "exit 7 (got $($r.Code)): $($r.Text)"
    Expect 'empty log' ($r.Text -match 'px4-stdout\.log') 'the text names px4-stdout.log'

    # stop fails
    $r = Run-Case 'stop fails' @{ FAKE_STOP_CODE = '1' } (Join-Path $work 'run-stop')
    Expect 'stop fails' ($r.Code -eq 7) "exit 7 (got $($r.Code)): $($r.Text)"
    Expect 'stop fails' ($r.Text -match 'docker stop') 'the text names docker stop'

    # cp fails
    $r = Run-Case 'cp fails' @{ FAKE_CP_CODE = '1' } (Join-Path $work 'run-cp')
    Expect 'cp fails' ($r.Code -eq 7) "exit 7 (got $($r.Code)): $($r.Text)"
    Expect 'cp fails' ($r.Text -match 'docker cp') 'the text names docker cp'

    # bad folder: a path below a regular file cannot be created
    $blocker = Join-Path $work 'blocker'
    Set-Content -LiteralPath $blocker 'a file'
    $r = Run-Case 'bad folder' @{} (Join-Path $blocker 'run')
    Expect 'bad folder' ($r.Code -eq 7) "exit 7 (got $($r.Code)): $($r.Text)"
    Expect 'bad folder' ($r.Text -match 'run folder') "the text names the run folder (got: $($r.Text))"

    # empty dir: a log folder with no .ulg in it is `none`, exit 0
    $r = Run-Case 'empty dir' @{ FAKE_CP_EMPTY = '1' } (Join-Path $work 'run-emptydir')
    Expect 'empty dir' ($r.Code -eq 0) "exit 0 (got $($r.Code)): $($r.Text)"
    Expect 'empty dir' ($r.Lines.Count -gt 0 -and $r.Lines[-1] -eq 'none') "the last line is none (got '$($r.Lines[-1])')"

    # start: the container's log folder is created before PX4 starts
    foreach ($k in 'FAKE_STOP_CODE', 'FAKE_CP_CODE', 'FAKE_RM_CODE', 'FAKE_RM_NOOP', 'FAKE_LOGS_EMPTY', 'FAKE_CP_EMPTY') { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
    $env:FAKE_CALLS = $calls; $env:FAKE_STATE = $state; $env:FAKE_LOGS_FILE = $logs; $env:FAKE_LABELS_JSON = '{}'
    Remove-Item -LiteralPath $calls, $state -ErrorAction SilentlyContinue
    $so = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -OutDir (Join-Path $work 'run-start') -Docker $fake 2>&1 | ForEach-Object { "$_" }
    $startCode = $LASTEXITCODE
    $runLine = @(Get-Content -LiteralPath $calls | Where-Object { $_ -match '^run ' }) -join ' '
    Expect 'start' ($startCode -eq 0) "exit 0 (got $startCode): $($so -join ' ')"
    Expect 'start' ($runLine -match 'mkdir -p \S*rootfs/log') "the run command creates the log folder first (got: $runLine)"
    Expect 'start' ($runLine -match 'exec \S*/bin/px4 \S*/etc -s etc/init\.d-posix/rcS -d') "and then execs px4 with its data dir and rcS (got: $runLine)"

    # start with -Network none: the run command carries `--network none` (the default is docker's own bridge, which a real run needs to reach the host)
    Remove-Item -LiteralPath $calls, $state -ErrorAction SilentlyContinue
    $so = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -OutDir (Join-Path $work 'run-start2') -Network none -Docker $fake 2>&1 | ForEach-Object { "$_" }
    $runLine2 = @(Get-Content -LiteralPath $calls | Where-Object { $_ -match '^run ' }) -join ' '
    Expect 'start network' ($LASTEXITCODE -eq 0) "exit 0 (got $LASTEXITCODE): $($so -join ' ')"
    Expect 'start network' ($runLine2 -match '--network none') "the run command carries --network none (got: $runLine2)"
    Expect 'start network' ($runLine -notmatch '--network') 'and the default start has no --network flag'

    # label: -Stop without -OutDir reads the run folder from the container's label (docker inspect ... json: no quotes inside the --format, which
    # Windows PowerShell 5.1 strips from a native command's arguments)
    $labelOut = Join-Path $work 'run-label'
    Remove-Item -LiteralPath $calls -ErrorAction SilentlyContinue
    Set-Content -LiteralPath $state 'x'
    foreach ($k in 'FAKE_STOP_CODE', 'FAKE_CP_CODE', 'FAKE_RM_CODE', 'FAKE_RM_NOOP', 'FAKE_LOGS_EMPTY', 'FAKE_CP_EMPTY') { Remove-Item "Env:$k" -ErrorAction SilentlyContinue }
    $env:FAKE_LABELS_JSON = (@{ 'spade.outdir' = $labelOut } | ConvertTo-Json -Compress)
    $lo = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -Stop -Docker $fake 2>&1 | ForEach-Object { "$_" }
    Expect 'label' ($LASTEXITCODE -eq 0) "exit 0 (got $LASTEXITCODE): $($lo -join ' ')"
    Expect 'label' (Test-Path -LiteralPath (Join-Path $labelOut 'px4-stdout.log')) 'the log is written into the folder named by the label'
    # no label, no -OutDir: exit 6, and the container is NOT removed (its log is the evidence)
    Set-Content -LiteralPath $state 'x'
    $env:FAKE_LABELS_JSON = '{}'
    $lo = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -Stop -Docker $fake 2>&1 | ForEach-Object { "$_" }
    Expect 'no label' ($LASTEXITCODE -eq 6) "exit 6 (got $LASTEXITCODE): $($lo -join ' ')"
    Expect 'no label' (Test-Path -LiteralPath $state) 'the container is left alone'
    Expect 'no label' (($lo -join ' ') -match 'docker rm -f spade-px4-t1') 'the text says how to remove it'

    # a relative run folder (from the label or from -OutDir) never becomes a path: exit 6, nothing written (a folder named {} once appeared in the worktree
    # when a fake label of {} was taken for one)
    $cwdBefore = @(Get-ChildItem -LiteralPath $RepoRoot -Force | ForEach-Object Name)
    Set-Content -LiteralPath $state 'x'
    $env:FAKE_LABELS_JSON = (@{ 'spade.outdir' = 'relative\dir' } | ConvertTo-Json -Compress)
    $lo = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -Stop -Docker $fake 2>&1 | ForEach-Object { "$_" }
    Expect 'relative label' ($LASTEXITCODE -eq 6) "exit 6 (got $LASTEXITCODE): $($lo -join ' ')"
    Expect 'relative label' (Test-Path -LiteralPath $state) 'the container is left alone'
    $env:FAKE_LABELS_JSON = '{}'
    Set-Content -LiteralPath $state 'x'
    $lo = & $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId t1 -Stop -OutDir 'rel\run' -Docker $fake 2>&1 | ForEach-Object { "$_" }
    Expect 'relative -OutDir' ($LASTEXITCODE -eq 6) "exit 6 (got $LASTEXITCODE): $($lo -join ' ')"
    $cwdAfter = @(Get-ChildItem -LiteralPath $RepoRoot -Force | ForEach-Object Name)
    Expect 'relative' (($cwdAfter -join '|') -eq ($cwdBefore -join '|')) "nothing was created in the repository root (before: $($cwdBefore -join ',') after: $($cwdAfter -join ','))"

    # no container
    $r = Run-Case 'no box' @{} (Join-Path $work 'run-nobox') -NoContainer
    Expect 'no box' ($r.Code -eq 0) "exit 0 (got $($r.Code))"
    Expect 'no box' ($r.Lines.Count -gt 0 -and $r.Lines[-1] -eq 'none') 'the last line is none'
}
finally {
    Remove-Item -Recurse -Force -LiteralPath $work -ErrorAction SilentlyContinue
}

if ($problems.Count -gt 0) {
    $problems | ForEach-Object { Write-Host "FAIL $_" }
    Write-Host "px4_sitl_test: $($problems.Count) problem(s)"
    exit 1
}
Write-Host 'px4_sitl_test: all cases pass'
exit 0
