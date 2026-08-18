<#
.SYNOPSIS
    Builds (if needed) and launches a Spade demo scene: either the v2 viewer
    (v2 physics through v1's frozen OpenGL stack) or a v1 regression scene
    (Sandbox.exe, unmodified v1 GL engine).

.DESCRIPTION
    Task 14's USER CHECKPOINT #1: `spade\scripts\demo.ps1 -Scene bounce` builds
    the requested target via build.ps1, then launches it with the mapped scene
    name as its one argument. build.ps1 runs in the FOREGROUND, mirroring
    build.ps1/test.ps1's own rule (backgrounded builds get killed on this
    box). Both targets are windowed apps: they open, render the scene, and
    are closed by the user (Esc or the window's close button).

    Task 14b amendment A3 added three more scenes -- v1-fluid, v1-spheres,
    v1-cubes -- which launch Sandbox.exe (spade/examples/sandbox) instead of
    spade_viewer.exe, so the pre-existing v1 renders stay runnable as visual
    regression references while v2 (spade/engine/) progresses. The four
    original scenes (drop, bounce, shower, gate) are unchanged and still
    launch spade_viewer.exe.

    A known cosmetic stderr line appears on this box during the build step
    below, and it is NOT a build failure -- but it is not build.ps1's own
    vswhere call that writes it (that call uses an absolute, Test-Path-
    guarded path to vswhere.exe and would throw its own clear error if it
    were the source). The line comes from INSIDE vcvars64.bat's own tooling,
    which build.ps1 invokes via `cmd /c` to import the MSVC environment.
    This script does not redirect build.ps1's stderr (no `2>&1`) so
    PowerShell never wraps that line into a terminating NativeCommandError
    under $ErrorActionPreference = 'Stop' -- it just checks $LASTEXITCODE
    afterward, exactly like build.ps1 checks cmake's.

.PARAMETER Scene
    Which demo scene to launch: drop, bounce, shower, gate, hover, wind,
    flight, swarm (spade_viewer, v2 physics), or v1-fluid, v1-spheres,
    v1-cubes (Sandbox, v1 regression references). Required.

.PARAMETER Preset
    Which build to launch: 'debug' or 'release', matching build.ps1's
    -Preset. Defaults to 'release'.

.PARAMETER Backend
    Which compute backend steps the v2 physics: 'cpu' (default) or 'vulkan'
    (S6 Task 6). Ignored for v1- scenes, which are Sandbox.exe regression
    references and have no v2 Simulation at all -- passing -Backend vulkan
    with one of those is an error rather than a silent no-op.

    'vulkan' routes the SAME scene, the same spawns and the same fixed-step
    decomposition through the ported compute kernels. ALL EIGHT schedule
    passes are ported as of S6 Task 8 -- MediumUpdate, ForceElements (rotors
    and drag), CollisionStatic, CollisionDynamic, Integrate and
    SensorSynthesis have kernels; Gravity and Publish are inert by design on
    both backends -- so no scene is refused for an unported pass any more.
    Through Task 7 a scene needing turbulence, rotors or IMU sensors was
    rejected at startup with a message naming the pass; that gate is gone.
    `bounce` remains the demonstration scene: four worlds, one ball each, a
    restitution ladder against a ground plane.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\demo.ps1 -Scene bounce
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\demo.ps1 -Scene bounce -Backend vulkan
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\demo.ps1 -Scene gate -Preset debug
.EXAMPLE
    powershell -ExecutionPolicy Bypass -File spade\scripts\demo.ps1 -Scene v1-fluid
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string] $Scene,

    [ValidateSet('debug', 'release')]
    [string] $Preset = 'release',

    [ValidateSet('cpu', 'vulkan')]
    [string] $Backend = 'cpu'
)

$ErrorActionPreference = 'Stop'

# v2 scenes (spade_viewer.exe) map 1:1 onto their own name; v1 scenes
# (Sandbox.exe) drop the "v1-" prefix to get Sandbox's own argv scene name
# (main.cpp: fluid|spheres|cubes).
#
# Task 20 added hover/wind/flight/swarm -- the flight demo + M1B verification
# scenes (spade/engine/tools/viewer/scenes.cpp's make_scene()/scene_names()).
$V2Scenes = @('drop', 'bounce', 'shower', 'gate', 'hover', 'wind', 'flight', 'swarm')
$V1SceneMap = @{
    'v1-fluid'   = 'fluid'
    'v1-spheres' = 'spheres'
    'v1-cubes'   = 'cubes'
}

$IsV1Scene = $V1SceneMap.ContainsKey($Scene)
if (-not $IsV1Scene -and ($V2Scenes -notcontains $Scene)) {
    Write-Host "Unknown scene '$Scene'." -ForegroundColor Red
    Write-Host "Valid scenes: $(($V2Scenes + $V1SceneMap.Keys) -join ', ')"
    exit 1
}

# -Backend is a v2-only knob: a v1- scene launches Sandbox.exe, which is v1's
# own unmodified renderer with no v2 Simulation in it at all. Rejecting the
# combination rather than ignoring it is the point -- silently dropping the
# switch would let someone believe they had just watched the Vulkan backend
# render a v1 regression scene.
#
# CHECKED HERE, BEFORE THE BUILD BELOW (S6 hygiene: T6 review M7 -- this used
# to run AFTER `& $BuildPs1`, so an invalid -Scene/-Backend combination paid
# for a full foreground build before being told it could never have launched
# anything). Both this check and the unknown-scene one above are pure
# argument validation with no dependency on a build having happened, so both
# now fail fast in the same place.
if ($IsV1Scene -and $Backend -ne 'cpu') {
    Write-Host "-Backend $Backend is not meaningful for the v1 scene '$Scene'." -ForegroundColor Red
    Write-Host "v1- scenes launch Sandbox.exe (v1's own renderer); they run no v2 physics."
    exit 1
}

# scripts/ is the child of spade/; spade/ is the CMake source dir.
$SpadeDir = Split-Path -Parent $PSScriptRoot
$BuildPs1 = Join-Path $PSScriptRoot 'build.ps1'

Write-Host "=== Building ($Preset) ===" -ForegroundColor Cyan
# NOTE: do not pipe/redirect build.ps1's stderr here -- see the header
# comment's vswhere note. Check $LASTEXITCODE instead.
& $BuildPs1 -Preset $Preset
if ($LASTEXITCODE -ne 0) {
    throw "build.ps1 failed (exit $LASTEXITCODE)"
}

# CMAKE_RUNTIME_OUTPUT_DIRECTORY (spade/CMakeLists.txt) puts every exe under
# <binaryDir>/bin; build-ninja/<preset> is CMakePresets.json's binaryDir for
# this preset (single-config Ninja generator -- no extra Debug/Release
# subfolder the way a multi-config generator would add). Sandbox.exe (v1,
# examples/sandbox/CMakeLists.txt's `add_executable(Sandbox ...)`) lands in
# the exact same bin/ as spade_viewer.exe -- one CMAKE_RUNTIME_OUTPUT_DIRECTORY
# for the whole build tree, both targets included.
if ($IsV1Scene) {
    $ExeName  = 'Sandbox.exe'
    $ExeArgs  = @($V1SceneMap[$Scene])
} else {
    $ExeName  = 'spade_viewer.exe'
    # The viewer's own argv contract (engine/tools/viewer/main.cpp):
    # <scene> [cpu|vulkan]. The backend argument is passed ALWAYS, not only
    # when it is 'vulkan', so the launched command line always states which
    # path it took.
    $ExeArgs  = @($Scene, $Backend)
}
$ExePath = Join-Path $SpadeDir "build-ninja/$Preset/bin/$ExeName"
if (-not (Test-Path $ExePath)) {
    throw "Executable not found at $ExePath after a successful build. " +
          "Was SPADE_BUILD_V1 enabled for this preset (it is ON by default)?"
}

Write-Host ""
Write-Host "=== Launching $ExeName : $Scene ($Preset) ===" -ForegroundColor Cyan
Write-Host "Exe      : $ExePath"
if (-not $IsV1Scene) {
    Write-Host "Backend  : $Backend"
}
if ($IsV1Scene) {
    Write-Host "Controls : WASD + Space/Shift to fly, M to start/stop the simulation (starts PAUSED), C to toggle mouse capture, Esc or close the window to quit."
} else {
    Write-Host "Controls : WASD + Space/Shift to fly, C to toggle mouse capture, Esc or close the window to quit."
}
Write-Host ""

# v1 loads shaders/assets via a CWD-relative "assets/shaders/..." path
# (Resources::LoadShaderFile) -- assets/ is copied next to the exe (root
# CMakeLists.txt's file(COPY ... DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}),
# NOT next to this script or the caller's own working directory. Launching
# with the caller's CWD (e.g. the repo root, per this script's own -Scene
# examples) fails at shader load with no window ever opening -- run from the
# exe's own directory instead. This applies identically to both exes: v1 is
# the ONLY renderer in play (spade_viewer bridges v2 physics into it), so
# both Sandbox.exe and spade_viewer.exe load assets the same CWD-relative way.
$ExeDir = Split-Path -Parent $ExePath
Push-Location $ExeDir
try {
    & $ExePath @ExeArgs
    $ViewerExitCode = $LASTEXITCODE
} finally {
    Pop-Location
}
exit $ViewerExitCode
