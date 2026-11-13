# Select, place and move

## Select something

1. Click the drone in the 3D view, or click its name in the **Hierarchy** on the left.
2. Its details appear in the **Inspector** on the right.
3. Press <kbd>F</kbd> to bring the camera in on it.

![A drone selected](images/editor_vehicle.png)

## Place something

1. In the **Library** at the bottom, click a picture, for example **Box** (under **primitives**). Its frame lights up and a line says "Click in the view to place Box."
2. Click on the ground in the 3D view where it should go.
3. The box appears there. It is selected and its name (for example `box`) is in the **Hierarchy**.
4. To place another, click the library picture again. One click places one thing. Click the same picture a second time before you click the ground to change your mind.

![Two boxes placed](images/editor_place.png)

If you click somewhere that is not ground (the sky, for example) the Inspector says "place 'Box': the click did not meet the ground; click on the ground where it should go", and nothing is placed.

## Move something

1. Select the object. Coloured arrows appear on it.
2. Make sure **Move** on the toolbar is chosen (or press <kbd>W</kbd>).
3. Put the mouse over an arrow until it lights up, then press and drag along it. The object moves in that direction only.
4. Let go. The whole drag is one step, so one click on **Undo** puts it back.

![Dragging an arrow](images/editor_gizmo.png)

## Turn something

1. Choose **Rotate** on the toolbar (or press <kbd>E</kbd>).
2. Drag one of the handles that appear on the object.

## Snap

Tick **Snap** on the toolbar, or hold <kbd>Ctrl</kbd> while you drag, to move and turn in steps. Holding <kbd>Ctrl</kbd> while Snap is ticked does the opposite.

## Copy and delete

1. With an object selected, click **Duplicate** at the bottom of the Inspector, or press <kbd>Ctrl</kbd>+<kbd>D</kbd>.
2. Click **Delete** or press <kbd>Delete</kbd> to remove it.

!!! tip "Try this"
    - Place ten boxes in a row. Do their names stay different?
    - Drag a box through the drone. What happens?
    - Press <kbd>Delete</kbd> with nothing selected.
    - Duplicate the drone and change one of the two. Does the other change too? (It should not.)
