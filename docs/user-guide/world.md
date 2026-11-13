# The world and the scene

The editor keeps two things apart:

- The **world** is the place: gravity, wind, air, light, the sky, where things may start.
- The **scene** is what you put in the world: your drones and objects.

The first two rows of the **Hierarchy** are headings for them: **Scene** and **World**.

## Change the world

1. Click the **World** row in the **Hierarchy** (it reads `World hover_ground` in the first scene).
2. The **Inspector** shows its settings:
    - **Environment**: **Gravity**, **Wind**, **Air density** and **Temperature**.
    - **Lighting**: where the sun is, its colour and strength, and the ambient light.
    - **Sky**, **Spawn points**, **Props**, **Capacities** and **Not saved with the world**: click the heading to open each.
3. Change a number. For example, set the first **Wind (m/s)** number to 2.
4. Click **Save** on the toolbar. The editor saves the world and then saves the scene so that it points at the saved world.

![The world's settings in the inspector](images/editor_world.png)

Note that **Wind** here is the world's own wind, saved with the world. **Wind now** while the scene runs is a separate, temporary wind.

## Look at the scene

1. Click the **Scene** row in the **Hierarchy**.
2. The **Inspector** shows which **World** file it uses, its **Materials** (the colours objects can have), **Spare slots**, and a button **Add material**.

## When the scene and the world disagree

If a change to one no longer fits the other, the editor says so at the top of the **Inspector**. It names what clashes. Click **Undo** to put it right.

![A message about a clash between the scene and the world](images/editor_conflict.png)

!!! tip "Try this"
    - Set **Gravity** to 0, then to a large number, then **Run**. What does the drone do?
    - Open **Capacities** and set a number to 0. What does the editor say?
    - Change the world, do not save, and open another scene with **File**. What happens to your change?
    - Click **Add material** a few times. Do the names stay different?
