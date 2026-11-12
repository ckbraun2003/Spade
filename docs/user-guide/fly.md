# Fly a drone

Any drone from the **Library** can be flown with the keyboard while the scene runs.

## Fly one

1. Place a drone (see [Add a drone](add-drone.md)). It is selected.
2. Click **Run**. The **Pilot** section appears in the **Inspector** and says "Flying phantom3_class, disarmed". It shows **Throttle**, **Pitch, roll (°)** and **Heading (°)**.
3. Press <kbd>Space</kbd>, or click **Arm**. It says "armed". The rotors turn slowly: this is idle.
4. Hold <kbd>Page Up</kbd> to raise the throttle. The drone climbs. <kbd>Page Down</kbd> lowers it.
5. Hold <kbd>Arrow Up</kbd> to tip the nose down and move forward. <kbd>Arrow Left</kbd> and <kbd>Arrow Right</kbd> roll it. <kbd>Z</kbd> and <kbd>X</kbd> turn it.
6. Press <kbd>R</kbd> (or click **Level**) to level it.
7. Press <kbd>Space</kbd> again (or **Disarm**) to stop the rotors.
8. Click **Stop**.

![A flying drone: the pilot section on the right](images/editor_fly_props.png)

## Which drone is flown

**Run** flies the drone that is selected. If you select a different drone before you click **Run**, that one is flown. If nothing is selected, it is the first drone that can fly. If none can, the Pilot section says why ("nothing to fly: ...").

## Camera while flying

Move the camera as usual: hold the right mouse button and drag to look around, and use the mouse wheel to come nearer (see [A first look](editor-first-look.md)).

!!! tip "Try this"
    - Hold <kbd>Page Up</kbd> until the drone is high, then let go.
    - Tip the drone a lot and let go of the keys.
    - Arm and disarm while the drone is in the air.
    - Fly into the ground, into a wall, and out past the edge of the ground.
    - Hold two opposite arrow keys at once.
    - Fly two drones: click **Stop**, select the other one, click **Run**.
