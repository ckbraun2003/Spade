<#
.SYNOPSIS
    Runs the Spade v2 engine test suite (spade_tests) via CTest against a
    build.ps1-built Ninja preset.

.DESCRIPTION
    Never invoke a test .exe directly -- gtest_discover_tests registers each
    TEST() as its own CTest case with LABELS "spade;T0" (spade/tests/
    CMakeLists.txt), and -L spade below relies on that registration to select
    the v2 engine suite.

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
