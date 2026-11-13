# Add objects

You can fill a scene with objects from the **Library**: **Box**, **Sphere**, **Cylinder**, **Capsule**, **Gate**, **Wall**, **Pole** and a floor tile. They are solid: a drone cannot fly through them.

## Place objects

1. Click **Gate** in the **Library**, then click the ground where it should stand.
2. Click **Wall**, then click the ground a little to one side.
3. Each stands upright on the ground at its normal size. Each name is in the **Hierarchy** (`gate`, `wall`, ...).

![A gate, a wall and boxes placed](images/editor_objects.png)

## Change an object

1. Click the object in the 3D view or in the **Hierarchy**.
2. In the **Inspector**:
    - **Transform**: **Position (m)**, **Rotation (°)** and **Scale**.
    - **Material**: the material (colour) of the **Visual** and of the **Collider** (the solid part). Click the list and choose another material.
    - **Collider**: how many pieces it is made of.
3. Drag the arrows in the 3D view to move it (see [Select, place and move](select-place-move.md)).
4. Click **Duplicate** or **Delete** at the bottom of the Inspector.

Objects you place stay where you put them. They do not fall or move by themselves.

!!! tip "Try this"
    - Make a slalom: place six poles in a row and fly a drone through.
    - Fly a drone into a wall, into a gate's frame, and through a gate.
    - Set **Scale** to 0, to a very large number, and to a negative number.
    - Place an object where a drone already is.
    - Put an object right on the edge of the ground.
    - Place fifty objects. Does the editor stay quick?
