<#
.SYNOPSIS
    Runs the Spade v2 engine test suite (spade_tests) via CTest against a
    build.ps1-built Ninja preset.

.DESCRIPTION
    Never invoke a test .exe directly -- gtest_discover_tests registers each
    TEST() as its own CTest case with LABELS "spade;T0" (spade/tests/
    CMakeLists.txt), and -L spade below relies on that registration to select
    the v2 engine suite.

    WHY -L spade AND NOT -L T0 (S6 hygiene note; no behavior change, this is
    the consumer decision the label's existence implies but nothing states):
    every test this suite registers carries BOTH labels today (spade/tests/
    AppendSpadeLabels.cmake's blanket "spade;T0" call), so the two selectors
    are currently equivalent in this build tree -- but they answer different
    questions, and only one of them is this script's job to answer. "spade"
    means "the v2 engine suite, whatever tier its tests happen to sit at";
    "T0" means "kat's tier-0 category" (docs/dev/testing.md's tier table --
    PR-gate, <=5 min budget), a cross-repo classification this suite happens
    to satisfy in full today but is not this script's to assert. T0 is
    RESERVED for a future kat-side spot-run consumer that selects across
    MULTIPLE C++ suites by tier (this one and interface/'s, ctest label `T1`
    for Qt-dependent code) rather than by which engine owns them -- that
    consumer does not exist in-repo yet, and when it does, it is the one that
    should invoke `ctest -L T0`, not this file. If spade's suite ever grows a
    genuinely slower category (a T1/T2 spade test), -L spade here keeps
    running everything unchanged; a future kat-side T0 spot-run would then,
    correctly, run fewer of this suite's tests than this script does.

.PARAMETER Preset
    Which build to test: 'debug' or 'release', matching build.ps1's -Preset.
    Defaults to 'release', i.e. spade/build-ninja/release.

.PARAMETER Filter
    Optional regex passed through to ctest's -R (only run matching test
    names).

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\test.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\test.ps1 -Preset debug
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\test.ps1 -Filter IntegrateOrientation
#>
[CmdletBinding()]
param(
    [ValidateSet('debug', 'release')]
    [string] $Preset = 'release',
    [string] $Filter
)

$ErrorActionPreference = 'Stop'

# scripts/ is the child of spade/; spade/build-ninja/<preset> is where
# CMakePresets.json's binaryDir puts this preset's build.
$SpadeDir = Split-Path -Parent $PSScriptRoot
$TestDir  = Join-Path $SpadeDir "build-ninja/$Preset"

if (-not (Test-Path (Join-Path $TestDir 'CMakeCache.txt'))) {
    throw "No configured build at $TestDir. Run build.ps1 -Preset $Preset first."
}

$ctestArgs = @('--test-dir', $TestDir, '-L', 'spade', '--output-on-failure', '--no-tests=error')
if ($Filter) {
    $ctestArgs += @('-R', $Filter)
}

Write-Host "Test dir : $TestDir"
Write-Host ("ctest    : {0}" -f ($ctestArgs -join ' '))
Write-Host ""

& ctest @ctestArgs
exit $LASTEXITCODE
