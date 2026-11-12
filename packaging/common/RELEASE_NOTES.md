# Release notes

## Spade Editor 0.1.0, the first tester release

This is the first version we are putting in front of testers. It works, and it has rough edges. This page says what is in it and, honestly, what is not.

### What is in it

**The drone box.** One quadcopter on a test stand. Tilt it with the arrow keys, change the wind, the air, the throttle, watch the numbers move.

**The editor.** A window with a toolbar on top, a list of the scene on the left, details on the right and a library of drones and objects at the bottom. In it you can:

- open one of the 14 built-in scenes, or one of your own files;
- click an object to select it, and drag its arrows to move or turn it, with Undo and Redo for every change;
- place drones and objects from the library by clicking on the ground: six drones, boxes, spheres, cylinders, capsules, a gate, a wall, a pole and a floor tile;
- change a drone's details, such as its rotors and starting position, and see where each number came from and which are only estimates;
- change the world (wind, gravity, air density, lighting, sky) and save it;
- look at the scene in different ways (shaded and other views from the View menu);
- press **Run** to make the scene move, and fly a drone with the keyboard;
- change the wind while the scene runs;
- press **Run air** to see the air a drone stirs up, at two levels of detail, with an optional extra for small swirls (Subgrid turbulence).

### What you should know before you start (known limits)

- **"Run air" is slow.** It shows the real airflow, and that costs a lot of work. At the Detailed level it ran at about 2% of real time on the graphics card in our test: less than one new picture a second. On the processor only (no Vulkan), it is slower again. This is not a bug; give it time, and use "Real time" for a quicker, rougher look.
- **"Run air" shows one drone and not the whole scene.** The other objects in the scene are not part of that run.
- **Wind during a run is separate from the airflow.** The "Wind now" row changes the wind of a normal **Run**. "Run air" keeps the wind it started with.
- **Subgrid turbulence needs the graphics card with Vulkan.** On the processor, it is switched off and the program says so.
- **Some numbers are estimates.** The library marks a drone or part "assumed" when a number in it is an estimate, not a measurement. Click it and the Inspector shows which.
- **Things the world file does not hold yet:** gusts, bounce, friction and the contact size are kept by the program while it is open, and are not saved with the world.
- **Placed objects are fixed.** Boxes, spheres, walls and gates you place stay where you put them. They do not fall, bounce or tumble. Only drones move.
- **Windows only, 64-bit.** There is no Mac or Linux version of this release.
- **Esc closes the window.** It does not ask whether you want to save. Use **Save** first.

### What is not in this release

- No version for other systems, and no automatic updates.

### Version

Spade Editor 0.1.0. The program shows the exact build [to confirm: where it shows it].
