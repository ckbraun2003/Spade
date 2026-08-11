<#
.SYNOPSIS
    Foreground Ninja build of the Spade v2 engine (spade/engine, spade/tests)
    via CMake presets.

.DESCRIPTION
    CMake's Ninja generator invokes cl.exe directly and does not set up the
    MSVC compiler environment itself, so this script imports it first (same
    vcvars64 discovery as interface/scripts/ninja-build.ps1), then configures
    (if not already configured) and builds the requested preset from
    spade/CMakePresets.json.

    Run this in the FOREGROUND. Backgrounded builds get killed on this box.

.PARAMETER Preset
    Which CMake preset to build: 'debug' or 'release' (msvc-ninja-debug /
    msvc-ninja-release). Defaults to 'release'.

.PARAMETER Clean
    Delete the preset's build directory before configuring, forcing a fresh
    configure and full rebuild.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\build.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\build.ps1 -Preset debug
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\build.ps1 -Preset release -Clean
#>
[CmdletBinding()]
param(
    [ValidateSet('debug', 'release')]
    [string] $Preset = 'release',
    [switch] $Clean
)

$ErrorActionPreference = 'Stop'

# scripts/ is the child of spade/; spade/ is the CMake source dir (where
# CMakePresets.json lives).
$SpadeDir   = Split-Path -Parent $PSScriptRoot
$PresetName = "msvc-ninja-$Preset"
$BuildDir   = Join-Path $SpadeDir "build-ninja/$Preset"

Write-Host "Spade  : $SpadeDir"
Write-Host "Preset : $PresetName"
Write-Host "Build  : $BuildDir"

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "Removing $BuildDir (-Clean)"
    Remove-Item -Recurse -Force $BuildDir
}

# ---------------------------------------------------------------------------
# 1. Locate ninja. Prefer PATH (winget install), then the winget package dir
#    (PATH is only updated for NEW shells, so a freshly-installed ninja is
#    invisible to the shell that installed it), then the copy bundled inside
#    Visual Studio as a last resort. Mirrors
#    interface/scripts/ninja-build.ps1's discovery logic.
# ---------------------------------------------------------------------------
$ninja = $null
$onPath = Get-Command ninja -ErrorAction SilentlyContinue
if ($onPath) {
    $ninja = $onPath.Source
} else {
    $candidates = @(
        (Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages\Ninja-build.Ninja_Microsoft.Winget.Source_8wekyb3d8bbwe\ninja.exe'),
        'C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe'
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { $ninja = $c; break }
    }
}
if (-not $ninja) {
    throw "ninja not found. Install with: winget install --id Ninja-build.Ninja --exact"
}
Write-Host "Ninja  : $ninja"

# Ninja must be on PATH for CMake's -G Ninja to find it during configure.
$ninjaDir = Split-Path -Parent $ninja
if ($env:PATH -notlike "*$ninjaDir*") { $env:PATH = "$ninjaDir;$env:PATH" }

# ---------------------------------------------------------------------------
# 2. Import the MSVC x64 environment. Ninja invokes cl.exe directly and will
#    fail configure ("no CMAKE_CXX_COMPILER could be found") without this.
#    vcvars64.bat only mutates its own cmd session, so run it and copy the
#    resulting variables back into this PowerShell process. Identical logic
#    to interface/scripts/ninja-build.ps1 -- read that script first if this
#    ever needs to change.
# ---------------------------------------------------------------------------
if (-not $env:VSCMD_ARG_TGT_ARCH) {
    $vswhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
    if (-not (Test-Path $vswhere)) {
        throw "vswhere.exe not found. Is Visual Studio installed?"
    }

    $vsRoot = & $vswhere -latest -products * `
        -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
        -property installationPath
    if (-not $vsRoot) { throw "No VS install with the MSVC x64 toolchain was found." }

    $vcvars = Join-Path $vsRoot 'VC\Auxiliary\Build\vcvars64.bat'
    if (-not (Test-Path $vcvars)) { throw "vcvars64.bat not found at $vcvars" }

    Write-Host "MSVC   : importing environment from $vcvars"

    # Build the command line as a plain string so no backtick-escaped quotes are
    # needed inside a PowerShell string literal.
    $vcvarsCmd = '"' + $vcvars + '" && set'
    cmd /c $vcvarsCmd | ForEach-Object {
        if ($_ -match '^([^=]+)=(.*)$') {
            Set-Item -Path "env:$($matches[1])" -Value $matches[2] -ErrorAction SilentlyContinue
        }
    }
    # vcvars prepends its own PATH; re-assert ninja so it stays reachable.
    if ($env:PATH -notlike "*$ninjaDir*") { $env:PATH = "$ninjaDir;$env:PATH" }
} else {
    Write-Host "MSVC   : environment already present (VSCMD_ARG_TGT_ARCH=$env:VSCMD_ARG_TGT_ARCH)"
}

# ---------------------------------------------------------------------------
# 3. Configure, only if this preset has not been configured yet (or -Clean
#    just removed it). CMakePresets.json pins generator, build type, and
#    binaryDir per preset, so no extra cache flags are needed here.
# ---------------------------------------------------------------------------
$cacheFile = Join-Path $BuildDir 'CMakeCache.txt'
if (-not (Test-Path $cacheFile)) {
    Write-Host ""
    Write-Host "=== Configuring ($PresetName) ===" -ForegroundColor Cyan
    & cmake -S $SpadeDir --preset $PresetName
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed (exit $LASTEXITCODE)" }
} else {
    Write-Host "Already configured. Skipping configure (pass -Clean to force a fresh one)."
}

# ---------------------------------------------------------------------------
# 4. Build.
# ---------------------------------------------------------------------------
Write-Host ""
Write-Host "=== Building ($PresetName) ===" -ForegroundColor Cyan
$sw = [System.Diagnostics.Stopwatch]::StartNew()
& cmake --build $BuildDir
if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)" }
$sw.Stop()
Write-Host ("Build completed in {0:N1}s" -f $sw.Elapsed.TotalSeconds) -ForegroundColor Green
