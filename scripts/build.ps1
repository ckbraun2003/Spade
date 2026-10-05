<#
.SYNOPSIS
    Foreground Ninja build of the Spade v2 engine (engine, tests)
    via CMake presets.

.DESCRIPTION
    CMake's Ninja generator invokes cl.exe directly and does not set up the
    MSVC compiler environment itself, so this script imports it first
    (vswhere -> vcvars64.bat), then configures (if not already configured)
    and builds the requested preset from CMakePresets.json.

    Run this in the FOREGROUND. Backgrounded builds get killed on this box.

.PARAMETER Preset
    Which CMake preset to build: 'debug' or 'release' (msvc-ninja-debug /
    msvc-ninja-release). Defaults to 'release'.

.PARAMETER Target
    Build a single target (spade_tests, spade_sandbox, spade_bench) instead of
    everything. One target per invocation, by design.

.PARAMETER BuildDir
    Build a tree this script did not lay out -- a worktree's own build
    directory, or a tree configured with non-preset flags such as
    -DSPADE_VULKAN=OFF. It must ALREADY be configured: the presets fix each
    preset's binaryDir, so this script refuses to configure a directory it
    cannot aim a configure at, rather than quietly configuring build-ninja
    and reporting success.

.PARAMETER ParallelLevel
    Compile jobs, 1..64, default 8. All-cores (0) is not expressible here: on
    the old, memory-bound box its failure mode was a silent OOM kill.

.PARAMETER Clean
    Delete the preset's build directory before configuring, forcing a fresh
    configure and full rebuild.

.NOTES
    THIS SCRIPT DRIVES build-ninja/<preset> BY DEFAULT. Any other tree --
    a worktree's, or one configured by hand -- is reached with -BuildDir,
    and a single target with -Target.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\build.ps1
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Preset debug
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -Preset release -Clean
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File scripts\build.ps1 -BuildDir build-novk -Target spade_tests
#>
[CmdletBinding()]
param(
    [ValidateSet('debug', 'release')]
    [string] $Preset = 'release',

    # Build ONE target instead of everything. Deliberately a single string and
    # not a [string[]]: the editor's wrapper takes an array and `-Target a,b`
    # fails there with no compiler output, so the shape that cannot work is
    # not offered here at all.
    [string] $Target,

    # Build a tree this script did not lay out -- a worktree's, or one
    # configured by hand. Must ALREADY be configured; see the refusal below
    # for why.
    [string] $BuildDir,

    # ValidateRange, not [int]: `--parallel 0` means ALL CORES, which the old
    # box (about 7.6 GB) answered with a silent OOM kill. Before this parameter
    # existed the script ran a bare `cmake --build`, ninja's all-cores default.
    # 8 suits the current machine (32 GB, 16 threads; 2026-10-04).
    [ValidateRange(1,64)]
    [int] $ParallelLevel = 8,

    [switch] $Clean
)

$ErrorActionPreference = 'Stop'

# scripts/ is the child of the repo root; the repo root is the CMake source
# dir (where CMakePresets.json lives).
$SpadeDir   = Split-Path -Parent $PSScriptRoot
$PresetName = "msvc-ninja-$Preset"

# -BuildDir aims this script at a tree the presets did not create. Without it
# the preset's own binaryDir is used, exactly as before.
$ExternalTree = [bool] $BuildDir
if (-not $ExternalTree) {
    $BuildDir = Join-Path $SpadeDir "build-ninja/$Preset"
} elseif (-not [System.IO.Path]::IsPathRooted($BuildDir)) {
    $BuildDir = Join-Path (Get-Location).Path $BuildDir
}

Write-Host "Spade  : $SpadeDir"
Write-Host "Preset : $PresetName"
Write-Host "Build  : $BuildDir"
if ($Target) { Write-Host "Target : $Target" } else { Write-Host "Target : <all>" }
Write-Host "Jobs   : $ParallelLevel"

if ($Clean -and (Test-Path $BuildDir)) {
    Write-Host "Removing $BuildDir (-Clean)"
    Remove-Item -Recurse -Force $BuildDir
}

# ---------------------------------------------------------------------------
# 1. Locate ninja. Prefer PATH (winget install), then the winget package dir
#    (PATH is only updated for NEW shells, so a freshly-installed ninja is
#    invisible to the shell that installed it), then the copy bundled inside
#    Visual Studio as a last resort.
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
#    resulting variables back into this PowerShell process.
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
    # A MISSING CAPABILITY REFUSES, IT DOES NOT DEGRADE. `cmake --preset` puts
    # its output in the PRESET's binaryDir and has no idea -BuildDir was
    # asked for, so configuring here would silently populate build-ninja and
    # then build a tree the caller never named -- a success message about the
    # wrong directory, which is the failure this script already caused once by
    # pointing at a tree that has never held spade_sandbox.
    if ($ExternalTree) {
        throw ("-BuildDir '$BuildDir' is not configured (no CMakeCache.txt), and this script " +
               "will not configure it: CMakePresets.json fixes each preset's binaryDir, so a " +
               "configure here would land in build-ninja/$Preset instead. Configure it yourself " +
               "with the cache flags that tree needs, then re-run.")
    }
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
$buildArgs = @('--build', $BuildDir, '--parallel', "$ParallelLevel")
if ($Target) { $buildArgs += @('--target', $Target) }
& cmake @buildArgs
if ($LASTEXITCODE -ne 0) { throw "Build failed (exit $LASTEXITCODE)" }
$sw.Stop()
Write-Host ("Build completed in {0:N1}s" -f $sw.Elapsed.TotalSeconds) -ForegroundColor Green
