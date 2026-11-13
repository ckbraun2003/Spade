# Views

A view is a way of drawing the scene. The normal one is **shaded**: lit, with shadows. The others show other things about the same scene.

## Change the view

1. On the toolbar, click the box next to **View** (it says `shaded`).
2. Click another name in the list:
    - **shaded**: the normal look.
    - **wireframe**: only the edges.
    - **velocity**: each object coloured by how fast it moves.
    - **exact_sdf**: the world drawn as exactly as the program knows it. This view is drawn by the processor only and shows no airflow.
    - **depth**: how far away each thing is, from near to far. A note on the toolbar says the range.
    - **normals**: which way each surface faces, as colours.
    - **segmentation**: each object in its own flat colour. Hover over the note on the toolbar to see which colour is which.
3. Choose **shaded** again to go back.

![The view list](images/editor_views.png)

A view the window cannot draw is greyed out, with the reason beside it. It is not drawn as an empty picture.

!!! tip "Try this"
    - Go through every view with the scene stopped, then again while it runs.
    - Choose **depth** and move the camera very close to an object, then very far away.
    - Choose a view while **Run air** is on. Do the airflow layers stay?
    - Change the view, then undo. Does the view change back? (It should not: a view is not an edit.)
