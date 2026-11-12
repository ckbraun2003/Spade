# System requirements

Spade Editor 0.1.0 is for Windows only.

## What you need

| | |
|---|---|
| **Windows** | Windows 10 or Windows 11, 64-bit |
| **Graphics card** | One that supports **OpenGL 4.3** (most graphics cards made in the last ten years do). Keep your graphics driver up to date. |
| **Vulkan (optional)** | A graphics card with a **Vulkan** driver lets "Run air" (the airflow around a drone) run at its best speed. Without it, everything else works and "Run air" runs on the processor, which is much slower. |
| **Screen** | 1280 x 720 or larger |
| **Internet** | Not needed to install or to run |
| **Administrator rights** | Not needed. Spade Editor installs for your Windows user only. |

## How much memory and disk it uses

These numbers are measured, not guessed. We measured them on one PC: Windows 10, 32 GB of memory, NVIDIA GeForce RTX 3060 Ti. They come from the status box in the bottom-left corner of the 3D view, in our automated test tour of build 669cb2b1.

| What you are doing | Memory the program uses |
|---|---|
| The drone box, open | about 80 MB |
| The editor, just opened | about 120 MB |
| The editor with several objects and a drone flying | about 175 MB |
| "Run air", Detailed, on the graphics card | about 450 MB (the most we measured) |

- **Disk space:** [to be filled from the installer's size, measured]
- **The lowest-memory PC we have tested:** [to be filled from the clean-PC run]

If your PC is below these, Spade Editor may still run, but we have not tested it there.

## Check your graphics card

1. Press the **Windows key**, type **dxdiag** and press **Enter**.
2. Open the **Display** tab and read the **Name** of your card and the **Driver Version**.
3. If the program says it needs OpenGL 4.3, update the graphics driver. Get it from the web site of your card maker (NVIDIA, AMD or Intel). Then start the program again.
