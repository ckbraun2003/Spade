# Inspect, undo and redo

## Look at a drone's details

1. Select a drone (click it in the 3D view or in the **Hierarchy**).
2. The **Inspector** shows its details in sections:
    - **Start**: where it begins (**Position**), which way it faces (**Attitude**), how fast it is moving (**Velocity**) and how fast the rotors turn (**Rotor speed**).
    - **Airflow**: the air around the drone (see [Run air](run-air.md)).
    - **Model**: the numbers that make this drone.
    - **Rotors** and other headings with a number after them: click the heading to open it.
    - **Advanced**: for the rarely used numbers; click it to open it.

![The inspector showing a drone's details](images/editor_inspect.png)

## Change a number

1. Click a number and drag left or right, or double-click it and type a new value, then press <kbd>Enter</kbd>.
2. The change applies when you let go.
3. It changes only the drone you selected. If you duplicated the drone, the copy keeps the old number.

## Undo and redo

1. Click **Undo** on the toolbar to take back your last change. A drag, a placement, a typed number each count as one.
2. Click **Redo** to bring it back.
3. When there is nothing to undo, the **Undo** button is greyed out. Same for **Redo**.

![Undo and Redo on the toolbar](images/editor_undo.png)

## Look at a library part

1. Click a picture in the **Library** that is marked **assumed** (a part, or a drone).
2. The **Inspector** lists each number, where it came from (a source, or "assumed", which means an estimate) and its value. Hover over a row to see the whole line.
3. A part (a propeller, a motor) cannot be placed on its own; it is built into a drone. Clicking a drone's picture also gets it ready to place: click the same picture again to cancel.

![A library drone inspected](images/editor_drone.png)

![A library part inspected](images/editor_part.png)

!!! tip "Try this"
    - Change the same number twenty times, then click **Undo** twenty times. Do you get back to where you started?
    - Click **Undo** until it is greyed out. Does anything go wrong?
    - Type a very large number, a negative number, or letters into a field.
    - Change a number, then click **Redo**. Does anything happen?
