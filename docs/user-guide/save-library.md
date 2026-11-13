# Save and the library

## Save your work

1. Click **Save** on the toolbar. The editor writes the scene file (and the world file it uses). The star after the scene's name on the toolbar goes away.
2. To keep the original, save under a new name first: click **File**, type a new path in **Scene file** (for example `C:\Users\you\Documents\my-scene.scene.yaml`), and click **Save As**.
3. To open your scene later, click **File**. Scenes you saved show in the **Scenes** list marked **(saved)**, or type the path in **Scene file** and click **Open**.

![Saving: the star after the name goes away](images/editor_save.png)

The editor does not save by itself, and <kbd>Esc</kbd> closes the window without asking.

## The library

The **Library** along the bottom lists what you can place, grouped by heading: **drones**, **primitives** (box, sphere, cylinder, capsule), **obstacles** (gate, wall, pole), **floors** (a floor tile), **test fixtures**, and the parts drones are built from: **props**, **motors**, **ESCs** and **batteries**.

- A drone or an object is placed by clicking its picture and then the ground.
- A part is for looking at only: click it and the Inspector says what it is.
- The word **assumed** under a picture means some number inside it is an estimate.

### Bring in your own

1. Type the path of a file in **Load from file** (an object file, or a 3D model file in glTF format).
2. Click **Load**. It joins the library, ready to place.

### Keep one of yours

1. Select an object or drone you placed.
2. Click **Save selection to library**. It is saved in a `library` folder next to your scene, and the Inspector says where.

### Use a 3D model as a drone's parts

1. Select a drone.
2. Type the path of a glTF file in **Load from file**.
3. Click **Import as the drone's parts**. The model's pieces become parts of that drone, and the Inspector shows each one under **Imported parts**.

!!! tip "Try this"
    - Click **Load** with the box empty, then with a path that does not exist, then with a file that is not a model.
    - Click **Save selection to library** with nothing selected. The Inspector should say what to do.
    - Click **Import as the drone's parts** with no drone selected.
    - Save a scene, change it, and open the saved scene again from **File**. Is it what you saved?
    - Use **Save As** with a path in a folder that does not exist.
