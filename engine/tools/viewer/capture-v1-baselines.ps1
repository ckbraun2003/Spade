<#
.SYNOPSIS
    Captures SL14b's perishable v1 baselines (INT-4): reference frames, process
    memory and the tools' own FPS lines for the eleven scenes the v1 tools show.

.DESCRIPTION
    Spec: docs/design/interface/plans/2026-10-02-v1-baselines.md, part A.

    v1 is frozen, so nothing here changes a tool. Each scene runs exactly as
    scripts/demo.ps1 runs it: the existing binary, started from bin/ (v1 loads
    its shaders relative to the working directory), with stdout captured. The
    harness then works from OUTSIDE the running process:

      frames   PrintWindow(PW_CLIENTONLY | PW_RENDERFULLCONTENT), which reads a
               GL window even when something covers it. Every frame is checked
               for blankness; a blank one is retaken by bringing the window to
               the top and copying it from the screen, and the method is
               recorded per frame.
      play     The v1 Sandbox starts paused and toggles play on EVERY frame the
               M key is down. The harness posts M key-down/key-up messages to
               the window (no focus needed: GLFW reads the scancode from the
               message), then checks that the scene MOVES by differencing two
               frames. One hold can toggle once or twice, so it retries with
               longer holds until motion is seen; after the last attempt it
               asks the person at the screen to tap M. A frame is never labelled
               "playing" unless motion was seen.
      memory   Get-Process private bytes and working set, once a second. The
               Sandbox prints its own "FPS: x | Mem: y MB" line every frame
               (PrivateUsage); those lines are kept as printed.
      close    WM_CLOSE, so the tool leaves its loop and exits cleanly.

    WHAT THE FRAMES ARE: wall-clock moments, not ticks. spade_viewer cannot say
    which tick it drew, and the v1 Sandbox is unseeded. They record what each
    scene LOOKS like -- geometry, motion between moments, materials -- which is
    SL14b's visual axis ("never a pixel diff").

    Output, per scene, under -OutDir:
      viewer/<scene>/   t0.png t1.png t3.png t8.png   stdout.txt  capture.json
      sandbox/<scene>/  paused.png play2.png play5.png play10.png
                        stdout.txt  capture.json
    t0 is the first non-blank frame; the others are seconds after it. For the
    Sandbox, playN is N seconds after motion was first seen.

    An existing scene directory is REFUSED unless -Force is given, in which case
    it is deleted first, so every capture lands in a fresh directory.

    Exit status: 0 every scene captured; 1 a scene failed (each is named);
    2 usage or refusal, before anything is launched.

.PARAMETER Scene
    Scenes to capture. Viewer scenes by name (drop bounce shower gate hover wind
    flight swarm); Sandbox scenes as sandbox:<name> (fluid spheres cubes).
    Default: all eleven.

.PARAMETER BinDir
    The built bin/ directory holding spade_viewer.exe, Sandbox.exe and assets/.
    Default: build-ninja/release/bin under this checkout.

.PARAMETER OutDir
    Where scene directories are written. Default: tests/golden/v1-baselines.

.PARAMETER Force
    Replace scene directories that already exist.

.EXAMPLE
    powershell -ExecutionPolicy Bypass -File engine\tools\viewer\capture-v1-baselines.ps1 -Scene drop,sandbox:spheres -OutDir $env:TEMP\v1-dry
#>
[CmdletBinding()]
param(
    [string[]] $Scene = @('drop', 'bounce', 'shower', 'gate', 'hover', 'wind', 'flight', 'swarm',
                          'sandbox:fluid', 'sandbox:spheres', 'sandbox:cubes'),
    [string] $BinDir,
    [string] $OutDir,
    [switch] $Force
)

$ErrorActionPreference = 'Stop'

$root = (Resolve-Path (Join-Path $PSScriptRoot '..\..\..')).Path
if (-not $BinDir) { $BinDir = Join-Path $root 'build-ninja\release\bin' }
if (-not $OutDir) { $OutDir = Join-Path $root 'tests\golden\v1-baselines' }

$viewerScenes = @('drop', 'bounce', 'shower', 'gate', 'hover', 'wind', 'flight', 'swarm')
$sandboxScenes = @('fluid', 'spheres', 'cubes')

function Refuse([string] $why) {
    Write-Host "capture-v1-baselines: $why"
    exit 2
}

# ---------------------------------------------------------------------------
# Resolve every scene and refuse BEFORE launching anything.
# ---------------------------------------------------------------------------
$plan = @()
foreach ($s in $Scene) {
    if ($s -like 'sandbox:*') {
        $name = $s.Substring(8)
        if ($sandboxScenes -notcontains $name) { Refuse "unknown Sandbox scene '$name' (fluid, spheres, cubes)" }
        $plan += [pscustomobject]@{ Tool = 'sandbox'; Name = $name; Exe = 'Sandbox.exe'; Args = @($name) }
    } else {
        if ($viewerScenes -notcontains $s) { Refuse "unknown scene '$s' (viewer: $($viewerScenes -join ' '); Sandbox: sandbox:fluid|spheres|cubes)" }
        $plan += [pscustomobject]@{ Tool = 'viewer'; Name = $s; Exe = 'spade_viewer.exe'; Args = @($s, 'cpu') }
    }
}
foreach ($p in $plan) {
    $exe = Join-Path $BinDir $p.Exe
    if (-not (Test-Path $exe)) { Refuse "$exe does not exist (build it, or pass -BinDir)" }
    $dir = Join-Path $OutDir (Join-Path $p.Tool $p.Name)
    if ((Test-Path $dir) -and -not $Force) { Refuse "$dir exists; pass -Force to replace it" }
}

# ---------------------------------------------------------------------------
# Win32: window capture, key posting, z-order. Physical pixels, so the bitmap
# matches the client area whatever the display scaling.
# ---------------------------------------------------------------------------
Add-Type -AssemblyName System.Drawing
Add-Type -ReferencedAssemblies System.Drawing -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class V1CaptureNative {
    [StructLayout(LayoutKind.Sequential)] public struct RECT { public int Left, Top, Right, Bottom; }
    [StructLayout(LayoutKind.Sequential)] public struct POINT { public int X, Y; }
    [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")] public static extern bool PrintWindow(IntPtr hwnd, IntPtr hdc, uint flags);
    [DllImport("user32.dll")] public static extern bool GetClientRect(IntPtr hwnd, out RECT r);
    [DllImport("user32.dll")] public static extern bool ClientToScreen(IntPtr hwnd, ref POINT p);
    [DllImport("user32.dll")] public static extern bool PostMessage(IntPtr hwnd, uint msg, IntPtr w, IntPtr l);
    [DllImport("user32.dll")] public static extern bool SetWindowPos(IntPtr hwnd, IntPtr after, int x, int y, int cx, int cy, uint flags);
}
'@
[void][V1CaptureNative]::SetProcessDPIAware()

$PW_CLIENT_FULL = 3          # PW_CLIENTONLY | PW_RENDERFULLCONTENT
$WM_KEYDOWN = 0x0100
$WM_KEYUP = 0x0101
$VK_M = 0x4D
$SCAN_M = 0x32
$SWP_KEEP = 0x0013           # SWP_NOSIZE | SWP_NOMOVE | SWP_NOACTIVATE

function Get-ClientSize([IntPtr] $hwnd) {
    $r = New-Object V1CaptureNative+RECT
    [void][V1CaptureNative]::GetClientRect($hwnd, [ref] $r)
    return @(($r.Right - $r.Left), ($r.Bottom - $r.Top))
}

# Samples a 64x36 grid of pixels: enough to tell blank from drawn, and moving
# from still, without walking every pixel through GetPixel.
function Get-Samples([System.Drawing.Bitmap] $bmp) {
    $out = New-Object 'System.Collections.Generic.List[int]'
    for ($j = 0; $j -lt 36; $j++) {
        for ($i = 0; $i -lt 64; $i++) {
            $x = [int](($i + 0.5) * $bmp.Width / 64)
            $y = [int](($j + 0.5) * $bmp.Height / 36)
            $out.Add($bmp.GetPixel($x, $y).ToArgb())
        }
    }
    return , $out
}

function Test-Blank($samples) {
    $first = $samples[0]
    $same = 0
    foreach ($v in $samples) { if ($v -eq $first) { $same++ } }
    return ($same -ge [int]($samples.Count * 0.995))
}

# Fraction of sampled pixels whose largest channel difference exceeds 24.
function Get-Motion($a, $b) {
    $changed = 0
    for ($k = 0; $k -lt $a.Count; $k++) {
        $x = $a[$k]; $y = $b[$k]
        $d = [Math]::Max([Math]::Abs((($x -shr 16) -band 255) - (($y -shr 16) -band 255)),
             [Math]::Max([Math]::Abs((($x -shr 8) -band 255) - (($y -shr 8) -band 255)),
                         [Math]::Abs(($x -band 255) - ($y -band 255))))
        if ($d -gt 24) { $changed++ }
    }
    return $changed / $a.Count
}

# One frame of the window's client area. Returns @{ Bitmap; Samples; Method }
# or $null when the window has no client area yet.
function Get-Frame([IntPtr] $hwnd) {
    $size = Get-ClientSize $hwnd
    if ($size[0] -le 0 -or $size[1] -le 0) { return $null }
    $bmp = New-Object System.Drawing.Bitmap $size[0], $size[1]
    $g = [System.Drawing.Graphics]::FromImage($bmp)
    $hdc = $g.GetHdc()
    [void][V1CaptureNative]::PrintWindow($hwnd, $hdc, $PW_CLIENT_FULL)
    $g.ReleaseHdc($hdc)
    $samples = Get-Samples $bmp
    $method = 'printwindow'
    if (Test-Blank $samples) {
        $pt = New-Object V1CaptureNative+POINT
        [void][V1CaptureNative]::ClientToScreen($hwnd, [ref] $pt)
        [void][V1CaptureNative]::SetWindowPos($hwnd, [IntPtr](-1), 0, 0, 0, 0, $SWP_KEEP)   # topmost
        Start-Sleep -Milliseconds 300
        $g.CopyFromScreen($pt.X, $pt.Y, 0, 0, $bmp.Size)
        [void][V1CaptureNative]::SetWindowPos($hwnd, [IntPtr](-2), 0, 0, 0, 0, $SWP_KEEP)   # not topmost
        $samples = Get-Samples $bmp
        $method = 'screen'
    }
    $g.Dispose()
    return @{ Bitmap = $bmp; Samples = $samples; Method = $method; Blank = (Test-Blank $samples) }
}

function Send-KeyHold([IntPtr] $hwnd, [int] $holdMs) {
    $down = [IntPtr]([int64](1 -bor ($SCAN_M -shl 16)))
    $up = [IntPtr]([int64]0xC0000001L -bor ([int64]$SCAN_M -shl 16))   # previous-state and transition bits set
    [void][V1CaptureNative]::PostMessage($hwnd, $WM_KEYDOWN, [IntPtr]$VK_M, $down)
    Start-Sleep -Milliseconds $holdMs
    [void][V1CaptureNative]::PostMessage($hwnd, $WM_KEYUP, [IntPtr]$VK_M, $up)
}

# ---------------------------------------------------------------------------
# One scene's run: a clock, a memory sampler, and the frames it saves.
# ---------------------------------------------------------------------------
$machine = [ordered]@{
    computer = $env:COMPUTERNAME
    os = (Get-CimInstance Win32_OperatingSystem).Caption
    gpu = @(Get-CimInstance Win32_VideoController | ForEach-Object { "$($_.Name) (driver $($_.DriverVersion))" })
}
$sourceHead = ''
try { $sourceHead = (& git -C $root rev-parse HEAD 2>$null) } catch { $sourceHead = '' }

function Invoke-Scene($p) {
    $dir = Join-Path $OutDir (Join-Path $p.Tool $p.Name)
    if (Test-Path $dir) { Remove-Item -Recurse -Force $dir }
    New-Item -ItemType Directory -Force $dir | Out-Null
    $exe = Join-Path $BinDir $p.Exe
    $stdout = Join-Path $dir 'stdout.txt'

    $rec = [ordered]@{
        scene = $p.Name; tool = $p.Tool; exe = $exe
        exe_sha256 = (Get-FileHash -Algorithm SHA256 $exe).Hash.ToLower()
        exe_mtime = (Get-Item $exe).LastWriteTimeUtc.ToString('o')
        args = $p.Args; source_head = $sourceHead; machine = $machine
        started_utc = (Get-Date).ToUniversalTime().ToString('o')
        frames = @(); memory = @(); notes = @()
    }
    $clock = [System.Diagnostics.Stopwatch]::StartNew()
    $proc = Start-Process -FilePath $exe -ArgumentList $p.Args -WorkingDirectory $BinDir -NoNewWindow -PassThru `
        -RedirectStandardOutput $stdout -RedirectStandardError (Join-Path $dir 'stderr.txt')
    $script:lastSample = -1000

    # Waits until $untilMs on the scene clock, sampling memory once a second and
    # failing if the tool exits on its own.
    function Wait-Until([int64] $untilMs) {
        while ($clock.ElapsedMilliseconds -lt $untilMs) {
            if ($proc.HasExited) { throw "the tool exited on its own (exit $($proc.ExitCode))" }
            if ($clock.ElapsedMilliseconds - $script:lastSample -ge 1000) {
                $proc.Refresh()
                $rec.memory += , @($clock.ElapsedMilliseconds, $proc.PrivateMemorySize64, $proc.WorkingSet64)
                $script:lastSample = $clock.ElapsedMilliseconds
            }
            Start-Sleep -Milliseconds 100
        }
    }
    function Save-Frame($frame, [string] $file) {
        $frame.Bitmap.Save((Join-Path $dir $file), [System.Drawing.Imaging.ImageFormat]::Png)
        $rec.frames += , [ordered]@{ file = $file; at_ms = $clock.ElapsedMilliseconds; method = $frame.Method }
    }

    try {
        # The window, then its first non-blank frame.
        $hwnd = [IntPtr]::Zero
        while ($hwnd -eq [IntPtr]::Zero) {
            if ($clock.ElapsedMilliseconds -gt 60000) { throw 'no window within 60 s' }
            Wait-Until ($clock.ElapsedMilliseconds + 200)
            $proc.Refresh()
            $hwnd = $proc.MainWindowHandle
        }
        $rec.window_ms = $clock.ElapsedMilliseconds
        $first = $null
        while ($null -eq $first) {
            if ($clock.ElapsedMilliseconds -gt 120000) { throw 'no non-blank frame within 120 s' }
            $f = Get-Frame $hwnd
            if ($null -ne $f -and -not $f.Blank) { $first = $f } else { Wait-Until ($clock.ElapsedMilliseconds + 500) }
        }
        $t0 = $clock.ElapsedMilliseconds
        $rec.first_frame_ms = $t0

        if ($p.Tool -eq 'viewer') {
            Save-Frame $first 't0.png'
            foreach ($s in 1, 3, 8) {
                Wait-Until ($t0 + 1000 * $s)
                $f = Get-Frame $hwnd
                if ($null -eq $f -or $f.Blank) { throw "frame t$s is blank" }
                Save-Frame $f "t$s.png"
            }
        } else {
            # Paused: the first frame, and proof that it is still.
            Save-Frame $first 'paused.png'
            Wait-Until ($clock.ElapsedMilliseconds + 1500)
            $still = Get-Frame $hwnd
            $rec.paused_motion = Get-Motion $first.Samples $still.Samples
            # A scene that moves before any M was sent would make every check
            # below meaningless: motion could not tell play from pause.
            if ($rec.paused_motion -gt 0.02) { throw "the scene moves before M was sent (motion $($rec.paused_motion)); play cannot be told from pause" }

            # Play: toggle until motion is seen.
            $attempts = @()
            $playing = $false
            foreach ($hold in 60, 120, 250, 500, 1000, 1500, 2000, 3000) {
                $a = Get-Frame $hwnd
                Send-KeyHold $hwnd $hold
                Wait-Until ($clock.ElapsedMilliseconds + 1500)
                $b = Get-Frame $hwnd
                $m = Get-Motion $a.Samples $b.Samples
                $attempts += , [ordered]@{ hold_ms = $hold; motion = $m }
                if ($m -gt 0.02) { $playing = $true; break }
            }
            if (-not $playing) {
                Write-Host ''
                Write-Host '  PRESS M ONCE NOW: click the Sandbox window and tap M once. Waiting up to 60 s.'
                Write-Host ''
                $deadline = $clock.ElapsedMilliseconds + 60000
                while (-not $playing -and $clock.ElapsedMilliseconds -lt $deadline) {
                    $a = Get-Frame $hwnd
                    Wait-Until ($clock.ElapsedMilliseconds + 1500)
                    $b = Get-Frame $hwnd
                    if ((Get-Motion $a.Samples $b.Samples) -gt 0.02) { $playing = $true; $rec.notes += 'play started by hand' }
                }
                if (-not $playing) { throw 'the scene never moved: play could not be started' }
            }
            $rec.play_attempts = $attempts
            $playMs = $clock.ElapsedMilliseconds
            $rec.play_seen_ms = $playMs
            $rec.play_stdout_line = @(Get-Content $stdout -ErrorAction SilentlyContinue).Count
            foreach ($s in 2, 5, 10) {
                Wait-Until ($playMs + 1000 * $s)
                $f = Get-Frame $hwnd
                if ($null -eq $f -or $f.Blank) { throw "frame play$s is blank" }
                Save-Frame $f "play$s.png"
            }
        }

        # Close the way a user would, and let stdout flush.
        [void]$proc.CloseMainWindow()
        if (-not $proc.WaitForExit(20000)) {
            $proc.Kill()
            $rec.notes += 'did not exit within 20 s of WM_CLOSE; killed'
        }
        $rec.exit_code = $proc.ExitCode
        $ok = $true
    } catch {
        $rec.failure = "$_"
        if (-not $proc.HasExited) { $proc.Kill() }
        $ok = $false
    }

    # The tools' own lines, summarised beside the raw file.
    $lines = @(Get-Content $stdout -ErrorAction SilentlyContinue)
    if ($p.Tool -eq 'sandbox') {
        $from = 0
        if ($rec.Contains('play_stdout_line')) { $from = $rec.play_stdout_line }
        $fps = @(); $mem = @()
        for ($k = $from; $k -lt $lines.Count; $k++) {
            if ($lines[$k] -match '^FPS: ([0-9.]+) \| Mem: ([0-9.]+) MB') { $fps += [double]$matches[1]; $mem += [double]$matches[2] }
        }
        $rec.stdout_during_play = [ordered]@{ lines = $fps.Count; fps_median = (Get-Median $fps); mem_mb_median = (Get-Median $mem) }
    } else {
        $fps = @()
        foreach ($l in $lines) { if ($l -match '^fps: ([0-9.]+) \|') { $fps += [double]$matches[1] } }
        $rec.stdout = [ordered]@{ lines = $fps.Count; fps_median = (Get-Median $fps) }
    }
    $rec.ok = $ok
    ($rec | ConvertTo-Json -Depth 6) | Out-File -Encoding utf8 (Join-Path $dir 'capture.json')
    return $ok
}

function Get-Median([double[]] $v) {
    if ($v.Count -eq 0) { return $null }
    $s = $v | Sort-Object
    return $s[[int][Math]::Floor(($s.Count - 1) / 2)]
}

$failed = @()
foreach ($p in $plan) {
    $label = "$($p.Tool)/$($p.Name)"
    Write-Host "capture-v1-baselines: $label ..."
    $t = Get-Date
    if (Invoke-Scene $p) {
        Write-Host ('capture-v1-baselines: {0} ok, {1:N0} s' -f $label, ((Get-Date) - $t).TotalSeconds)
    } else {
        $why = (Get-Content (Join-Path $OutDir "$($p.Tool)\$($p.Name)\capture.json") -Raw | ConvertFrom-Json).failure
        Write-Host "capture-v1-baselines: FAIL $label -- $why"
        $failed += $label
    }
}
if ($failed.Count -gt 0) {
    Write-Host "capture-v1-baselines: $($failed.Count) scene(s) failed: $($failed -join ', ')"
    exit 1
}
Write-Host "capture-v1-baselines: all $($plan.Count) scene(s) captured into $OutDir"
exit 0
