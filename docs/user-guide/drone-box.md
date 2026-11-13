# The drone box

The drone box is one quadcopter on a test stand. It stays in place, so you can tilt it, change the air around it and read what its rotors are doing, without it flying off.

## Open it

1. Press the **Windows key**, type **Spade Drone Box** and press **Enter**.
2. A window opens with the drone in the middle and a panel called **Drone** on the right.

![The drone box](images/01_drone_box.png)

## Tilt and turn the drone

1. Hold <kbd>Arrow Up</kbd>: the nose tips down. Hold <kbd>Arrow Down</kbd>: it tips up.
2. Hold <kbd>Arrow Left</kbd> or <kbd>Arrow Right</kbd> to roll it.
3. Hold <kbd>Z</kbd> or <kbd>X</kbd> to turn it left or right.
4. Let go of the keys. The drone holds the angle you gave it. The **Readouts** in the panel show **Attitude** (where it is) and **Target** (where you asked it to be). They end up the same.
5. Press <kbd>R</kbd> to level it. Within a couple of seconds both angles come back to about zero.

## Move the camera

1. Hold the left or right mouse button on the 3D view and drag: the camera goes around the drone.
2. <kbd>A</kbd> and <kbd>D</kbd> go around, <kbd>E</kbd> and <kbd>Q</kbd> go over and under, <kbd>W</kbd> and <kbd>S</kbd> come nearer and go further. It will not come closer or go further than a set distance.

## Change the air and the throttle

All of these are in the **Drone** panel, under **Air and world**.

1. Drag **Wind** between 0 and 20 m/s. The change applies when you let go of the slider.
2. Drag **Heading** to choose the direction of the wind, from 0 to 360 degrees.
3. Open **Turbulence** and choose **none**, **light**, **moderate** or **severe**.
4. Drag **Throttle** between 0% and 200% of hover. At 100% the rotors spin at the speed that would hold the drone up. Watch the **Rotor** lines under **Readouts**: each shows its speed in rad/s and its thrust in newtons.

## Settings you rarely need

Click **Advanced** at the bottom of the panel to open it.

- **Air density** and **Gravity**: change them and watch **Hover** and the rotor speeds change.
- **Backend**: **CPU** or **Vulkan**. This chooses where the drone's physics is calculated. If your computer has no Vulkan, the panel says why and the drone stays on the CPU.

## Show or hide the numbers

- <kbd>F1</kbd> hides or shows the small box in the bottom-left corner (frames a second, milliseconds a frame, and memory used). Click **Controls** in it to see the key list.
- <kbd>Esc</kbd> closes the window.

![The status box open, with the drone's backend named](images/drone_box_hud.png)

!!! tip "Try this"
    - Set **Throttle** to 0% and then to 200%. What do the rotor lines say? Does the drone stay on the stand?
    - Tilt the drone and set **Wind** to 20 m/s at different **Heading** values. Does the **Moment (cmd)** line change?
    - Switch **Backend** to **Vulkan** and back while the drone is tilted. Does the tilt stay the same?
