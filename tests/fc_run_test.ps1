<#
.SYNOPSIS
    scripts\fc-run.ps1 runs the autopilot tests the way docs/design/test-docs/plans/2026-11-21-fc-tests.md section 2 says, with a fake docker-up, a fake
    px4-sitl.ps1 and a fake ctest on PATH: no Docker, no PX4 and no real test is touched, so it runs anywhere.

.DESCRIPTION
    Cases:
      pass        ctest passes: exit 0, `fc run: PASS`; the container was started with the run id and the host address, stopped once, after the tests;
                  the tests saw SPADE_FC_RUN_ID, SPADE_FC_HOST_ADDR and SPADE_FC_OUT_DIR; the autopilot log's path is in the summary
      fail        ctest exits non-zero: exit 1, `FAIL`, and the container is still stopped
      skipped     ctest exits 0 but a test skipped: exit 2, `NOT GREEN`, the test named (a skip never counts toward green)
      no image    px4-sitl.ps1 exits 2 with a reason: exit 2, the reason and every `fc` test named as not run, ctest not run, nothing to stop
      start 3/4/5/7  px4-sitl.ps1 exits 3 (docker unreachable), 4 (not up; PX4's log lines kept), 5 (run id in use; that container is never stopped) or
                  anything else: exit 1, FAIL, the reason named, ctest not run
      stop none   -Stop prints `none` (no .ulg): a pass, and the summary says PX4 wrote no log
      stop 3/6    -Stop exits 3 (container may still be running: how to remove it is named) or 6 (no run folder): exit 1
      stop gone   the container was gone at -Stop: a pass becomes NOT GREEN, exit 2
      no daemon   docker-up.ps1 fails: exit 1, px4-sitl.ps1 never called
      no tests    ctest finds no `fc` test (exit 8): exit 1, container stopped (--no-tests=error is passed)
      bad run id  a run id with a slash: exit 1, nothing called
      filter      -Filter reaches ctest as -R
      k10         nothing fc-run wrote into the run folder (summary.txt, ctest.log, ctest-junit.xml) holds the user's folder: the autopilot log's path is
                  relative to the run folder and the ctest output goes through the installer records' redaction (ConvertTo-PublicRecord). The test runs with a
                  FAKE profile folder (USERPROFILE, LOCALAPPDATA, APPDATA moved under its own temp folder) that holds the build tree and the run folder, so
                  the leak shows whatever the real user is called; a leak is any file that still names the test's whole temp folder.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File tests\fc_run_test.ps1 <repo root>
#>
param([Parameter(Mandatory)] [string] $RepoRoot)
$ErrorActionPreference = 'Continue'
$problems = New-Object System.Collections.Generic.List[string]
$Error.Clear()
# $Ok may be a bool or the array that -match returns on an array (any match is a pass).
function Expect([string] $Case, [object] $Ok, [string] $What) {
    $good = if ($Ok -is [array]) { $Ok.Count -gt 0 } else { [bool] $Ok }
    if (-not $good) { $problems.Add("${Case}: $What") | Out-Null }
}

$script = Join-Path $RepoRoot 'scripts\fc-run.ps1'
$ps = Join-Path $PSHOME 'powershell.exe'
$work = Join-Path ([IO.Path]::GetTempPath()) ('spade-fc-run-test-' + [guid]::NewGuid().ToString('N').Substring(0, 12))
New-Item -ItemType Directory -Force -Path $work | Out-Null

$savedFolders = @{ USERPROFILE = $env:USERPROFILE; LOCALAPPDATA = $env:LOCALAPPDATA; APPDATA = $env:APPDATA }
try {
    # A fake user profile inside the test's own folder: the build tree and the run folder live under it, as they do under the real one.
    $fakeHome = Join-Path $work 'Users\Ada'
    $env:USERPROFILE = $fakeHome
    $env:LOCALAPPDATA = Join-Path $fakeHome 'AppData\Local'
    $env:APPDATA = Join-Path $fakeHome 'AppData\Roaming'
    $fake = Join-Path $work 'fake'
    $build = Join-Path $fakeHome 'build'
    $outRoot = Join-Path $fakeHome 'out'
    New-Item -ItemType Directory -Force -Path $fake, $build, $outRoot | Out-Null
    Set-Content -LiteralPath (Join-Path $build 'CTestTestfile.cmake') '# fake'
    $calls = Join-Path $work 'calls.txt'
    $ctestEnv = Join-Path $work 'ctest-env.txt'

    # A fake docker-up.ps1: exits FAKE_UP_CODE.
    Set-Content -LiteralPath (Join-Path $fake 'docker-up.ps1') @'
Add-Content -LiteralPath $env:FAKE_CALLS 'docker-up'
exit ([int] $env:FAKE_UP_CODE)
'@
    # A fake px4-sitl.ps1 with the contract of Interface's scripts\px4-sitl.ps1 (b558905e): start prints one line and exits 0, or a reason and
    # 2 no image, 3 docker unreachable, 4 not up (its log follows the first line), 5 run id in use; -Stop (OutDir optional) exits 0 with its LAST
    # line the bare newest .ulg path or `none` ("no container named" first when it is gone), 3 docker unreachable, 6 no run folder known.
    # FAKE_PX4_START_CODE and FAKE_PX4_STOP (ulg | none | gone | <exit code>) steer it.
    Set-Content -LiteralPath (Join-Path $fake 'px4-sitl.ps1') @'
param([string] $RunId, [string] $HostAddress, [string] $OutDir, [switch] $Stop)
if ($Stop) {
    Add-Content -LiteralPath $env:FAKE_CALLS "stop $RunId"
    $mode = $env:FAKE_PX4_STOP
    if ($mode -eq '3') { Write-Output 'px4-sitl: docker is not reachable (is Docker Desktop running?)'; exit 3 }
    if ($mode -eq '6') { Write-Output 'px4-sitl: no run folder known (pass -OutDir)'; Write-Output 'none'; exit 6 }
    if ($mode -eq 'gone') { Write-Output "px4-sitl: no container named spade-px4-$RunId"; Write-Output 'none'; exit 0 }
    if ($mode -eq 'none') { Write-Output 'none'; exit 0 }
    Write-Output (Join-Path $OutDir 'px4-log\2026-11-21\px4.ulg')
    exit 0
}
Add-Content -LiteralPath $env:FAKE_CALLS "start $RunId $HostAddress"
switch ([int] $env:FAKE_PX4_START_CODE) {
    0 { Write-Output "px4-sitl: up spade-px4-$RunId -> ${HostAddress}:4560"; exit 0 }
    2 { Write-Output 'px4-sitl: no autopilot image spade-px4-sitl:v1.17.0 (build it from tools/fc_link/px4)'; exit 2 }
    3 { Write-Output 'px4-sitl: docker is not reachable (is Docker Desktop running?)'; exit 3 }
    4 { Write-Output "px4-sitl: spade-px4-$RunId exited at start; its log follows"; Write-Output 'ERROR [px4] no such file: etc/init.d-posix/rcS'; Write-Output "INFO [px4] log folder $OutDir"; exit 4 }
    5 { Write-Output "px4-sitl: a container named spade-px4-$RunId already exists (one container per run id)"; exit 5 }
    default { Write-Output 'px4-sitl: something nobody planned'; exit ([int] $env:FAKE_PX4_START_CODE) }
}
'@
    # A fake ctest: -N lists two fc tests; otherwise it records the environment and prints FAKE_CTEST_BODY, then exits FAKE_CTEST_CODE.
    [IO.File]::WriteAllText((Join-Path $fake 'ctest.cmd'), @"
@echo off
echo ctest %*>>"%FAKE_CALLS%"
for %%a in (%*) do if "%%a"=="-N" goto list
echo %SPADE_FC_RUN_ID%^|%SPADE_FC_HOST_ADDR%^|%SPADE_FC_OUT_DIR%>"%FAKE_CTEST_ENV%"
set "NEXTJ="
for %%a in (%*) do call :scan "%%~a"
type "%FAKE_CTEST_BODY%"
exit /b %FAKE_CTEST_CODE%
:scan
if defined NEXTJ (type "%FAKE_CTEST_BODY%">%1)
set "NEXTJ="
if "%~1"=="--output-junit" set "NEXTJ=1"
goto :eof
:list
echo Test project fake
echo   Test #1: FcLiveLink.LockstepTenThousandSteps
echo   Test #2: FcLiveFlight.ScriptedFlight
echo Total Tests: 2
exit /b 0
"@.Replace("`r`n", "`n").Replace("`n", "`r`n"))

    $env:PATH = "$fake;$env:PATH"
    $env:FAKE_CALLS = $calls
    $env:FAKE_CTEST_ENV = $ctestEnv
    $body = Join-Path $work 'ctest-body.txt'
    $env:FAKE_CTEST_BODY = $body

    function Run([string] $Case, [string] $RunId, [int] $UpCode, [int] $StartCode, [int] $CtestCode, [string] $CtestBody, [string[]] $Extra = @(), [string] $StopMode = 'ulg') {
        Remove-Item -LiteralPath $calls, $ctestEnv -ErrorAction SilentlyContinue
        Set-Content -LiteralPath $body -Value $CtestBody
        $env:FAKE_UP_CODE = "$UpCode"
        $env:FAKE_PX4_STOP = $StopMode
        $env:FAKE_PX4_START_CODE = "$StartCode"
        $env:FAKE_CTEST_CODE = "$CtestCode"
        $text = (& $ps -NoProfile -ExecutionPolicy Bypass -File $script -RunId $RunId -Build $build -OutRoot $outRoot -HostAddress '10.0.0.7' `
                -DockerUp (Join-Path $fake 'docker-up.ps1') -Px4Script (Join-Path $fake 'px4-sitl.ps1') @Extra 2>&1 | Out-String)
        $code = $LASTEXITCODE
        $callLines = if (Test-Path -LiteralPath $calls) { @(Get-Content -LiteralPath $calls) } else { @() }
        $summaryPath = Join-Path (Join-Path $outRoot $RunId) 'summary.txt'
        $summary = if (Test-Path -LiteralPath $summaryPath) { (Get-Content -LiteralPath $summaryPath -Raw) } else { '' }
        return [pscustomobject]@{ Code = $code; Text = $text; Calls = $callLines; Summary = $summary }
    }

    # The files of a run folder that still name the test's whole temp folder (it holds the fake home), whichever slash and case.
    function LeakedFiles([string] $Folder) {
        $needle = $work.Replace('\', '/').ToLowerInvariant()
        @(Get-ChildItem -LiteralPath $Folder -Recurse -File | Where-Object { [IO.File]::ReadAllText($_.FullName).Replace('\', '/').ToLowerInvariant().Contains($needle) } | ForEach-Object { $_.Name })
    }

    $passBody = "Test project $build`n1/2 Test #1: FcLiveLink.LockstepTenThousandSteps ... Passed`n2/2 Test #2: FcLiveFlight.ScriptedFlight ... Passed`n`n100% tests passed, 0 tests failed out of 2"

    # pass
    $r = Run 'pass' 'case-pass' 0 0 0 $passBody
    Expect 'pass' ($r.Code -eq 0) "exit $($r.Code): $($r.Text)"
    Expect 'pass' ($r.Summary.StartsWith('fc run: PASS')) "summary: $($r.Summary)"
    Expect 'pass' ($r.Calls -contains 'start case-pass 10.0.0.7') "no start with the run id and host address: $($r.Calls -join ' / ')"
    Expect 'pass' (@($r.Calls | Where-Object { $_ -eq 'stop case-pass' }).Count -eq 1) 'the container was not stopped exactly once'
    $calls1 = @($r.Calls)
    $iStart = [array]::IndexOf($calls1, 'start case-pass 10.0.0.7')
    $iCtest = -1
    for ($i = 0; $i -lt $calls1.Count; $i++) { if ($calls1[$i] -like 'ctest *') { $iCtest = $i; break } }
    $iStop = [array]::IndexOf($calls1, 'stop case-pass')
    Expect 'pass' ($iStart -ge 0 -and $iStart -lt $iCtest -and $iCtest -lt $iStop) "order: start $iStart, ctest $iCtest, stop $iStop"
    Expect 'pass' ($r.Calls[0] -eq 'docker-up') 'docker-up did not come first'
    Expect 'pass' ($r.Calls -match '^ctest .*-L fc') 'ctest was not given -L fc'
    Expect 'pass' ($r.Calls -match 'no-tests=error') 'ctest was not given --no-tests=error'
    $envLine = (Get-Content -LiteralPath $ctestEnv -ErrorAction SilentlyContinue | Select-Object -First 1)
    Expect 'pass' ($envLine -eq ('case-pass|10.0.0.7|' + (Join-Path $outRoot 'case-pass'))) "the tests saw: $envLine"
    Expect 'pass' ($r.Summary.Contains('px4.ulg')) "the autopilot log path is not in the summary: $($r.Summary)"
    Expect 'pass' ($r.Summary.Contains('tests skipped: 0')) 'no skipped count'
    Expect 'pass' (Test-Path -LiteralPath (Join-Path (Join-Path $outRoot 'case-pass') 'ctest.log')) 'no ctest.log in the run folder'
    # k10: the autopilot log's path is relative to the run folder; what ctest printed and wrote has the home folder redacted (and still has its line).
    $passFolder = Join-Path $outRoot 'case-pass'
    Expect 'k10' ($r.Summary -match '(?m)^autopilot log: px4-log\\2026-11-21\\px4\.ulg\r?$') "the autopilot log is not given relative to the run folder: $($r.Summary)"
    $leaks = @(LeakedFiles $passFolder)
    Expect 'k10' ($leaks.Count -eq 0) "the user's folder is still in: $($leaks -join ', ')"
    $ctestLog = if (Test-Path -LiteralPath (Join-Path $passFolder 'ctest.log')) { Get-Content -LiteralPath (Join-Path $passFolder 'ctest.log') -Raw } else { '' }
    Expect 'k10' ($ctestLog.Contains('Test project %USERPROFILE%\build')) "ctest.log lost or kept its path line: $ctestLog"
    $junit = if (Test-Path -LiteralPath (Join-Path $passFolder 'ctest-junit.xml')) { Get-Content -LiteralPath (Join-Path $passFolder 'ctest-junit.xml') -Raw } else { '' }
    Expect 'k10' ($junit.Contains('Test project %USERPROFILE%\build')) "ctest-junit.xml is missing or kept its path: $junit"

    # fail
    $r = Run 'fail' 'case-fail' 0 0 8 "1/2 Test #1: FcLiveLink.LockstepTenThousandSteps ... ***Failed`n`n50% tests passed, 1 tests failed out of 2"
    Expect 'fail' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'fail' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"
    Expect 'fail' ($r.Calls -contains 'stop case-fail') 'the container was not stopped after a failed test'
    $leaks = @(LeakedFiles (Join-Path $outRoot 'case-fail'))
    Expect 'k10' ($leaks.Count -eq 0) "a failed run's folder still names the user's folder in: $($leaks -join ', ')"

    # skipped
    $r = Run 'skipped' 'case-skip' 0 0 0 "1/2 Test #1: FcLiveLink.LockstepTenThousandSteps ... Passed`n2/2 Test #2: FcLiveFlight.ScriptedFlight ...***Skipped   0.01 sec`n`n100% tests passed, 0 tests failed out of 2"
    Expect 'skipped' ($r.Code -eq 2) "exit $($r.Code): a skip must not count toward green"
    Expect 'skipped' ($r.Summary.StartsWith('fc run: NOT GREEN')) "summary: $($r.Summary)"
    Expect 'skipped' ($r.Summary.Contains('skipped: FcLiveFlight.ScriptedFlight')) "the skipped test is not named: $($r.Summary)"
    Expect 'skipped' ($r.Calls -contains 'stop case-skip') 'the container was not stopped'

    # no image
    $r = Run 'no image' 'case-noimg' 0 2 0 $passBody
    Expect 'no image' ($r.Code -eq 2) "exit $($r.Code)"
    Expect 'no image' ($r.Summary.StartsWith('fc run: NOT GREEN')) "summary: $($r.Summary)"
    Expect 'no image' ($r.Summary.Contains('no autopilot image spade-px4-sitl:v1.17.0')) "the reason is not in the summary: $($r.Summary)"
    Expect 'no image' ($r.Summary.Contains('not run: FcLiveLink.LockstepTenThousandSteps') -and $r.Summary.Contains('not run: FcLiveFlight.ScriptedFlight')) "the tests are not named as not run: $($r.Summary)"
    Expect 'no image' (-not ($r.Calls -match '^ctest .*-L fc --output')) 'the tests ran with no autopilot'
    Expect 'no image' (-not ($r.Calls -contains 'stop case-noimg')) 'a container that never started was stopped'

    # px4-sitl.ps1's other start exits (3 docker unreachable, 4 not up, 5 id in use, anything else): each is a FAIL with its own named reason,
    # ctest is not run, and a container this run did not start is never stopped.
    $r = Run 'start 3' 'case-s3' 0 3 0 $passBody
    Expect 'start 3' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'start 3' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"
    Expect 'start 3' ($r.Summary.Contains('Docker was not reachable')) "the reason is not named: $($r.Summary)"
    Expect 'start 3' (-not ($r.Calls -match '^(ctest .*-L fc --output|stop)')) "ctest or stop ran: $($r.Calls -join ' / ')"

    $r = Run 'start 4' 'case-s4' 0 4 0 $passBody
    Expect 'start 4' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'start 4' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"
    Expect 'start 4' ($r.Summary.Contains('did not stay up')) "the reason is not named: $($r.Summary)"
    Expect 'start 4' ($r.Summary.Contains('no such file: etc/init.d-posix/rcS')) "PX4's own log lines are not in the summary: $($r.Summary)"
    Expect 'start 4' (-not ($r.Calls -match '^(ctest .*-L fc --output|stop)')) "ctest or stop ran: $($r.Calls -join ' / ')"
    Expect 'start 4' ($r.Summary.Contains('INFO [px4] log folder')) "PX4's own log lines are not in the summary: $($r.Summary)"
    $leaks = @(LeakedFiles (Join-Path $outRoot 'case-s4'))
    Expect 'k10' ($leaks.Count -eq 0) "a failed start's summary still names the user's folder in: $($leaks -join ', ')"

    $r = Run 'start 5' 'case-s5' 0 5 0 $passBody
    Expect 'start 5' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'start 5' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"
    Expect 'start 5' ($r.Summary.Contains('already in use') -and $r.Summary.Contains('-RunId')) "the reason is not named: $($r.Summary)"
    Expect 'start 5' (-not ($r.Calls -contains 'stop case-s5')) 'a container this run did not start was stopped'

    $r = Run 'start 7' 'case-s7' 0 7 0 $passBody
    Expect 'start 7' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'start 7' ($r.Summary.Contains('exited 7')) "the unexpected exit is not named: $($r.Summary)"

    # px4-sitl.ps1 -Stop: its last line is `none` when PX4 wrote no log (a pass still, and said so); exit 3 leaves the container running (a FAIL that says
    # how to remove it); exit 6 is no run folder (a FAIL); a container already gone makes a pass NOT GREEN.
    $r = Run 'stop none' 'case-none' 0 0 0 $passBody @() 'none'
    Expect 'stop none' ($r.Code -eq 0) "exit $($r.Code): $($r.Text)"
    Expect 'stop none' ($r.Summary.Contains('autopilot log: none (PX4 wrote no .ulg)')) "the missing log is not said plainly: $($r.Summary)"

    $r = Run 'stop 3' 'case-st3' 0 0 0 $passBody @() '3'
    Expect 'stop 3' ($r.Code -eq 1) "exit $($r.Code): a container left running must not pass"
    Expect 'stop 3' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"
    Expect 'stop 3' ($r.Summary.Contains('may still be running') -and $r.Summary.Contains('docker rm -f spade-px4-case-st3')) "the leftover container is not named: $($r.Summary)"

    $r = Run 'stop 6' 'case-st6' 0 0 0 $passBody @() '6'
    Expect 'stop 6' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'stop 6' ($r.Summary.Contains('no run folder')) "the reason is not named: $($r.Summary)"

    $r = Run 'stop gone' 'case-gone' 0 0 0 $passBody @() 'gone'
    Expect 'stop gone' ($r.Code -eq 2) "exit $($r.Code): a pass with no container at the end is not green"
    Expect 'stop gone' ($r.Summary.StartsWith('fc run: NOT GREEN')) "summary: $($r.Summary)"
    Expect 'stop gone' ($r.Summary.Contains('was gone when the tests ended')) "the vanished container is not named: $($r.Summary)"

    # no daemon
    $r = Run 'no daemon' 'case-nodocker' 1 0 0 $passBody
    Expect 'no daemon' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'no daemon' (-not ($r.Calls -match '^(start|stop|ctest)')) "something ran with no daemon: $($r.Calls -join ' / ')"
    Expect 'no daemon' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"

    # no tests
    $r = Run 'no tests' 'case-notests' 0 0 8 'No tests were found!!!'
    Expect 'no tests' ($r.Code -eq 1) "exit $($r.Code): no tests must not be a pass"
    Expect 'no tests' ($r.Calls -contains 'stop case-notests') 'the container was not stopped'

    # bad run id
    $r = Run 'bad run id' 'bad/id' 0 0 0 $passBody
    Expect 'bad run id' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'bad run id' ($r.Calls.Count -eq 0) "something was called with a bad run id: $($r.Calls -join ' / ')"

    # filter
    $r = Run 'filter' 'case-filter' 0 0 0 $passBody @('-Filter', 'FcLiveFlight')
    Expect 'filter' ($r.Code -eq 0) "exit $($r.Code)"
    Expect 'filter' ($r.Calls -match '^ctest .*-R FcLiveFlight') 'the filter did not reach ctest as -R'
}
finally {
    $env:USERPROFILE = $savedFolders.USERPROFILE; $env:LOCALAPPDATA = $savedFolders.LOCALAPPDATA; $env:APPDATA = $savedFolders.APPDATA
    Remove-Item -LiteralPath $work -Recurse -Force -ErrorAction SilentlyContinue
}

# An exception the cases themselves raised (a bad parameter, a null) is a failure, not a quiet pass; native stderr is not counted.
$unexpected = @($Error | Where-Object { $_.FullyQualifiedErrorId -notmatch 'NativeCommandError|PathNotFound|ItemNotFound' })
foreach ($e in ($unexpected | Select-Object -First 3)) { $problems.Add("the test script raised: $e") | Out-Null }
if ($problems.Count -gt 0) {
    $problems | ForEach-Object { Write-Host "FAIL $_" }
    exit 1
}
Write-Host 'fc_run_test: all cases hold'
exit 0
