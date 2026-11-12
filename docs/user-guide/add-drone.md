# Add a drone

The library has six drones: **Phantom 3-class quad**, **Iris+-class quad**, **5-inch FPV quad**, **Crazyflie-class micro**, **Basic quad, 6-inch** and **Basic quad, competition**. Each has a motor, a propeller and a battery, so each can fly. Under **test fixtures** there is one more, **Reference quad (test fixture)**: a plain drone we use for our own checks. You can place it too.

## Place one

1. In the **Library**, under **drones**, click a picture, for example **Phantom 3-class quad**. The Inspector shows where each of its numbers came from, and a line under the toolbar says "Click in the view to place Phantom 3-class quad."
2. Click on the ground in the 3D view.
3. The drone rests on the ground, and its name (for example `phantom3_class`) is in the **Hierarchy**, selected.

![A drone placed and selected](images/editor_placed_drone.png)

4. Click **Run**. The drone stays still on the ground, because it is not armed yet. To fly it, see [Fly a drone](fly.md).
5. Click **Stop**. To take the drone out again, click **Undo**.

## See where a drone's numbers came from

1. Click the drone's picture in the **Library**.
2. The Inspector lists every number with its **source** or the word **assumed**. Under **Built from** it names the parts: the motor, the propeller, the battery.

To change a placed drone's numbers, see [Inspect, undo and redo](inspect-undo.md).

!!! tip "Try this"
    - Place all six drones, one after the other, and **Run**. Do they all stand still on the ground?
    - Place a drone on top of a box. Does it stand on the box?
    - Place a drone in the air by clicking the sky. What does the editor say?
    - Place the same drone twice and change the rotor numbers of one. Does the other stay as it was?
    - Place a drone, **Run**, and move it with the arrows while it runs.
