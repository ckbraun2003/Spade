# Try this

We want you to find the odd corners. Each chapter ends with a few things to try. This page collects the ones that cut across the whole program. For each, we say what should happen. If something else happens, that is what we want to know about.

## The window

1. **Resize the window** to very small, then very large, while the editor shows a scene. The panels and the 3D view should stay usable, and nothing should turn black or stretch.
2. **Minimise and restore** the window while **Run** is on, then while **Run air** is on. It should carry on.
3. **Drag a panel** by its title (**Hierarchy**, **Inspector**, **Library**) to a new place, or let it float. Press <kbd>F2</kbd>: the panels should go back where they started.
4. **Move the window to a second screen**, or change Windows' display scaling (Settings, Display, Scale) to 150% and start the program again. Text should stay readable.
5. **Open the editor and the drone box at the same time.** Both should work.
6. **Start the program twice.** Each copy should work on its own.

## Saving and files

1. **Change something, then press <kbd>Esc</kbd>.** The window closes without asking. Your change is lost. (This is known; we want to know if it surprises you.)
2. **Save As** to a path with spaces, accents or non-English letters in it, then open it again from **File**.
3. **Open** a path that does not exist, a folder, and a file that is not a scene. The Inspector should say what is wrong, and the program should carry on.
4. **Open the same scene twice** in a row, and many different scenes one after another. Does the memory number in the status box (bottom-left of the 3D view) keep growing?
5. **Save, change, Undo**: after you click **Undo**, is the star after the scene's name still there? Should it be?

## Running

1. **Leave Run on for ten minutes.** Does the memory number stay flat? Does the drone stay put?
2. **Leave Run air on** and lock the PC, or let it sleep, then come back.
3. **Run air on a laptop with two graphics chips** (an Intel one and an NVIDIA or AMD one). Which one does it use? If you can choose in Windows' graphics settings, try both.
4. **Remote Desktop:** start the program inside a Remote Desktop session. The program may refuse to open a window. If it does, it should say why.
5. **Run, Pause, Step, Run, Stop** in every order you can think of.

## Odd input

1. **Type** a letter, a very large number, a negative number or nothing into every number you can find.
2. **Drag** a number far past what looks reasonable.
3. **Click very fast**: on **Undo**, **Redo**, **Run**, **Stop**, **Duplicate**, **Delete**.
4. **Hold two opposite keys** (arrow left and arrow right, **Page Up** and **Page Down**).
5. **Scroll the mouse wheel** in the 3D view all the way in and all the way out.

## When something goes wrong

If the program closes by itself, or says something you do not understand, open the log. The newest file in `%LOCALAPPDATA%\Spade\logs` is the log of that run. See [Install and uninstall](install.md).
