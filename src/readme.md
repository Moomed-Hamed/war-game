### update notes
+ TODO : improve lighting system
+ TODO : bullet physics integration

### 🎮 main.cpp
- init window, input, game systems, and timer
- main game loop
- - update window & input
- - handle key input
- - update game state
- - update renderer
- - render scene
- - update frame timer & restart loop
- shutdown window & exit

### 😡 enemies.h
green spheres that turn more red as they take damage and explode when destroyed

### 🔫 weps.h

#### bullets
bullets have a type, damage, and age(for despawning)
#### weapon types
Gun_Meta: struct containing information for each weapon type
#### weapon animation
every weapon has it's own animation function that handles all the visual aspects of the animation
such as keyframe selection and blending based on the state of the weapon

in general, weapon animations are grouped by wep type (smg, rifle, bolt action, etc),but depending
on the wep there may be custom animations

### 🧙 player.h
player is just a camera glued to a physics sphere

### 🛢 props.h
handles static prop behavior & rendering

- crates : wooden, break into planks
- tanktraps : metal barrier
- sandbag : sand barrier
- campfire : place to cook fish
- tree : cover, can be chopped down
- barrel : explosive

### 💥 particles.h
a particle is defined as...

### ⛰ terrain.h
defines the structure that stores all the heightmap data for the game map as well as some functions
for manipulating it like explode()

also loads heightmaps from .R32 files

### 🧲 physics.h
has some utility functions for detecting collisions

### 🎨 renderer.h
+ TODO : try to at least document the animation stuff

### ⚙️ window.h
Handles opening a window, setting up OpenGL & OpenAL, and handling keyboard and mouse input

### ⚙️ boilerplate.h
contains all external libraries & includes + mathematics.h, contains macros for printing to
console & allocating memory; contains code for file reading, timers, multithreading, and playing audio

external libraries:
bullet physics: modified to not need libraries, includes only