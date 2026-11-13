# Run

**Run** makes the scene move: the drones obey gravity and the controls, and the world's wind blows. It does not draw the air. For that, see [Run air](run-air.md).

## Start, pause and stop

1. Click **Run** on the toolbar. The button becomes **Stop**, and the counter beside it (`tick 0`) starts to count.
2. Click **Pause** to freeze the scene, and **Resume** to carry on.
3. While it is paused, click **Step** to move it forward by one small step at a time.
4. Click **Stop** to end the run. The scene goes back to how you had it.

![A scene running](images/editor_running.png)

## Fly the selected drone

While the scene runs, the **Inspector** shows a **Pilot** section. It is for the selected drone, or for the first drone that can fly if none is selected.

1. Press <kbd>Space</kbd>, or click **Arm**, to start the rotors. The button now says **Disarm**.
2. Hold <kbd>Page Up</kbd> and <kbd>Page Down</kbd> to raise and lower the throttle.
3. Use the arrow keys to tilt, <kbd>Z</kbd> and <kbd>X</kbd> to turn, and <kbd>R</kbd> or **Level** to level.

More on flying is in [Fly a drone](fly.md). If a drone cannot fly (for example a plain box), the Pilot section says why.

## Change the wind while it runs

1. While the scene runs (not **Run air**), look in the **Inspector** for **Wind now**.
2. Change the three **Wind (m/s)** numbers (the wind along x, along y, which is up, and along z).
3. The wind changes from the next moment. The drone is pushed.

This wind is for this run only. It is not saved in the scene, and the **World** panel's own **Wind** is not changed. See [the world and the scene](world.md).

![The Wind now row while a scene runs](images/editor_wind.png)

If you try this with nothing running, the Inspector says "Wind: only a running scene has a live wind; Run it first". With **Run air** on, it says "Wind: the airflow keeps its start wind; Stop it and Run to change the wind".

## Edit while it runs

You can move an object or change a number while the scene runs. The run carries on from where each drone was.

!!! tip "Try this"
    - Click **Run**, then click **Run** again, then **Stop** twice.
    - Pause, press **Step** many times, then **Resume**.
    - Run, then click **Run air** (it is greyed out) or press **Delete** on the drone you are flying.
    - Set the wind to a large number, then to a negative number.
    - Run with no drone in the scene (delete them first).
