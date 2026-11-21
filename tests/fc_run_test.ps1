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
      no image    px4-sitl.ps1 exits 3 with a reason: exit 2, the reason and every `fc` test named as not run, ctest not run, nothing to stop
      no daemon   docker-up.ps1 fails: exit 1, px4-sitl.ps1 never called
      no tests    ctest finds no `fc` test (exit 8): exit 1, container stopped (--no-tests=error is passed)
      bad run id  a run id with a slash: exit 1, nothing called
      filter      -Filter reaches ctest as -R

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

try {
    $fake = Join-Path $work 'fake'
    $build = Join-Path $work 'build'
    $outRoot = Join-Path $work 'out'
    New-Item -ItemType Directory -Force -Path $fake, $build, $outRoot | Out-Null
    Set-Content -LiteralPath (Join-Path $build 'CTestTestfile.cmake') '# fake'
    $calls = Join-Path $work 'calls.txt'
    $ctestEnv = Join-Path $work 'ctest-env.txt'

    # A fake docker-up.ps1: exits FAKE_UP_CODE.
    Set-Content -LiteralPath (Join-Path $fake 'docker-up.ps1') @'
Add-Content -LiteralPath $env:FAKE_CALLS 'docker-up'
exit ([int] $env:FAKE_UP_CODE)
'@
    # A fake px4-sitl.ps1: logs its calls, fails to start with FAKE_PX4_START_CODE (a reason on its last line), and prints a log path when it stops.
    Set-Content -LiteralPath (Join-Path $fake 'px4-sitl.ps1') @'
param([string] $RunId, [string] $HostAddress, [string] $OutDir, [switch] $Stop)
if ($Stop) {
    Add-Content -LiteralPath $env:FAKE_CALLS "stop $RunId"
    Write-Output 'stopping the container'
    Write-Output (Join-Path $OutDir 'px4.ulg')
    exit 0
}
Add-Content -LiteralPath $env:FAKE_CALLS "start $RunId $HostAddress"
if ([int] $env:FAKE_PX4_START_CODE -ne 0) {
    Write-Output 'checking the image'
    Write-Output 'the image spade-px4-sitl:v1.17.0 is missing'
    exit ([int] $env:FAKE_PX4_START_CODE)
}
Write-Output 'started'
exit 0
'@
    # A fake ctest: -N lists two fc tests; otherwise it records the environment and prints FAKE_CTEST_BODY, then exits FAKE_CTEST_CODE.
    [IO.File]::WriteAllText((Join-Path $fake 'ctest.cmd'), @"
@echo off
echo ctest %*>>"%FAKE_CALLS%"
for %%a in (%*) do if "%%a"=="-N" goto list
echo %SPADE_FC_RUN_ID%^|%SPADE_FC_HOST_ADDR%^|%SPADE_FC_OUT_DIR%>"%FAKE_CTEST_ENV%"
type "%FAKE_CTEST_BODY%"
exit /b %FAKE_CTEST_CODE%
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

    function Run([string] $Case, [string] $RunId, [int] $UpCode, [int] $StartCode, [int] $CtestCode, [string] $CtestBody, [string[]] $Extra = @()) {
        Remove-Item -LiteralPath $calls, $ctestEnv -ErrorAction SilentlyContinue
        Set-Content -LiteralPath $body -Value $CtestBody
        $env:FAKE_UP_CODE = "$UpCode"
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

    $passBody = "1/2 Test #1: FcLiveLink.LockstepTenThousandSteps ... Passed`n2/2 Test #2: FcLiveFlight.ScriptedFlight ... Passed`n`n100% tests passed, 0 tests failed out of 2"

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

    # fail
    $r = Run 'fail' 'case-fail' 0 0 8 "1/2 Test #1: FcLiveLink.LockstepTenThousandSteps ... ***Failed`n`n50% tests passed, 1 tests failed out of 2"
    Expect 'fail' ($r.Code -eq 1) "exit $($r.Code)"
    Expect 'fail' ($r.Summary.StartsWith('fc run: FAIL')) "summary: $($r.Summary)"
    Expect 'fail' ($r.Calls -contains 'stop case-fail') 'the container was not stopped after a failed test'

    # skipped
    $r = Run 'skipped' 'case-skip' 0 0 0 "1/2 Test #1: FcLiveLink.LockstepTenThousandSteps ... Passed`n2/2 Test #2: FcLiveFlight.ScriptedFlight ...***Skipped   0.01 sec`n`n100% tests passed, 0 tests failed out of 2"
    Expect 'skipped' ($r.Code -eq 2) "exit $($r.Code): a skip must not count toward green"
    Expect 'skipped' ($r.Summary.StartsWith('fc run: NOT GREEN')) "summary: $($r.Summary)"
    Expect 'skipped' ($r.Summary.Contains('skipped: FcLiveFlight.ScriptedFlight')) "the skipped test is not named: $($r.Summary)"
    Expect 'skipped' ($r.Calls -contains 'stop case-skip') 'the container was not stopped'

    # no image
    $r = Run 'no image' 'case-noimg' 0 3 0 $passBody
    Expect 'no image' ($r.Code -eq 2) "exit $($r.Code)"
    Expect 'no image' ($r.Summary.StartsWith('fc run: NOT GREEN')) "summary: $($r.Summary)"
    Expect 'no image' ($r.Summary.Contains('the image spade-px4-sitl:v1.17.0 is missing')) "the reason is not in the summary: $($r.Summary)"
    Expect 'no image' ($r.Summary.Contains('not run: FcLiveLink.LockstepTenThousandSteps') -and $r.Summary.Contains('not run: FcLiveFlight.ScriptedFlight')) "the tests are not named as not run: $($r.Summary)"
    Expect 'no image' (-not ($r.Calls -match '^ctest .*-L fc --output')) 'the tests ran with no autopilot'
    Expect 'no image' (-not ($r.Calls -contains 'stop case-noimg')) 'a container that never started was stopped'

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
