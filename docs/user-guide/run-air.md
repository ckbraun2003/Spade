# Run air

**Run air** shows the air a drone moves as it flies: the stream pushed down by the rotors, the swirls that peel off, and how fast the air goes. It is the slowest thing in the program, because it works out the air all around the drone. Give it time.

## Run the air around a drone

1. Select a drone in the **Hierarchy** (place one from the **Library** first if you like).
2. In the **Inspector**, find the **Airflow** section and click **Add airflow region**. This marks the box of air that follows the drone. The section now shows the region's name, a **Physics** list, a line about the air's grid, and a button **Remove airflow region**.
3. Leave **Physics** at **Detailed** for now.
4. Click **Run air** on the toolbar.
5. The toolbar shows the level you chose and how fast the run goes compared with real time. Lines and swirl shapes appear around the drone, with a key for them in the bottom-right of the 3D view.
6. Press <kbd>Space</kbd> to arm the drone and hold <kbd>Page Up</kbd> to throttle up, as in [Fly a drone](fly.md). The **Flight** section of the Inspector shows the air speed, the angles the air meets the drone at, and each rotor's thrust against the drone's weight.
7. Click **Stop** to end the run.

![Run air on a drone](images/editor_air_running.png)

## Choose how detailed

Open the **Physics** list under **Airflow** (you cannot change it while the air runs):

| Level | What it does | Cost |
|---|---|---|
| **Detailed** | Puts the drone's body in the air, as well as its rotors. | The wake is within 4% of a much slower reference run, and the load on the body within about 40%. Slow: about 2% of real time on the graphics card in our test. |
| **Real time** | Rotors only, as flat discs. | Rougher on the pressure, quicker. |

The sentence under the list says it again, in the program's own words.

## Small swirls: Subgrid turbulence

1. Select the drone and open **Advanced** under its **Airflow** section.
2. Tick **Subgrid turbulence**.
3. Click **Run air**. A line under the row reads like "Cs 0.17 · nu_t up to X m2/s; nothing at the cap or the wall limit".

This needs a graphics card with Vulkan. Without one, the run goes on without it and says "subgrid turbulence needs the GPU". You cannot tick it while the air runs.

![The Subgrid turbulence row](images/editor_subgrid.png)

## If Run air says no

| The editor says | Do this |
|---|---|
| "Run air: no drone here has an airflow region; select a drone and Add airflow region" | Select a drone, click **Add airflow region**. |
| "Run air: the scene is running; Stop it first" | Click **Stop**, then **Run air**. |
| "the air stopped: the scene changed; Run air again" | You edited the scene while the air ran. Click **Run air** again. |
| "Run: the air is running; Stop it first" | Click **Stop** before **Run**. |

!!! tip "Try this"
    - Run air on each of the six drones.
    - Run air at **Detailed** and at **Real time**. Compare the speed shown on the toolbar.
    - Edit the scene (move a box) while the air runs.
    - Click **Stop** straight away, and click **Run air** again many times.
    - Resize or minimise the window while the air runs.
    - Run air on a computer without Vulkan: how slow is it, and does the program stay alive?
