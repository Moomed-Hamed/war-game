# Architecture
---

## One header per file

**Rule:** each header includes *only* the next one down the chain.

Every header in `src/` includes exactly the one below it, so `main.cpp`
effectively pulls the entire engine into one compilation unit:

**Why?**
- To keep things simple and efficient.
- No include guards, no link-order problems, and one obvious dependency direction.

---

## Header Chain

dependencies:

| File | Purpose |
|---|---|
| `boilerplate.h` | Includes every external library; the game only needs this file at the start |

src:

| File | Purpose |
|---|---|
| `logger.h` | In-game console log |
| `loader.h` | Generated mesh primitives, texture loading, shader programs |
| `audio.h` | Sound effects and music |
| `window.h` | Window, input, timing, g-buffer, HDR targets, tonemap, audio device |
| `drawer.h` | Draw call preparation, light volumes, forward water |
| `physics.h` | Bullet world, rigid bodies, constraints, soft bodies |
| `player.h` | Capsule movement and the free fly camera |
| `fluids.h` | FFT ocean, foam field, short-chop normals, shallow water patch |
| `scene.h` | Scene content, shared builders, physics/render/water scenes, scene manager |
| `ux.h` | Console, scene selector, debug tools |
| `main.cpp` | Main loop |

---

## Frame pipeline

1. `GameRenderer::draw_shadows` renders the static world into the sun depth map.
2. `GameRenderer::draw` fills the g-buffer (position, normal, albedo, depth).
3. `GameWindow::draw_lighting` runs the sun (PCF shadowed), ambient, sky and fog into a linear HDR target.
4. `GameRenderer::draw_light_volumes` adds the point lights, one instanced volume per light.
5. `GameRenderer::draw_water` copies the lit frame and draws forward water with refraction, absorption, sky reflection, sun specular, crest scattering, breaking shore foam and a CC0 foam lace mask.
6. `GameWindow::post_process` applies exposure, contrast, saturation, ACES and gamma to the screen.
7. `debug_draw` appends physics, light-volume and scene-guide wires into one colour-coded overlay.

## Current scope

The shipping project is now a rendering/physics/water foundation: scene
content, the deferred renderer, Bullet rigid and soft bodies, the FFT ocean and
the shallow-water patch. The procedural city, building generator, hub, weapons
scene, terrain/world scene and vehicle scene have been removed from the build.
Their research and source snapshots are under `megastudy/procgen-city/` and
`megastudy/terrain-archive/`.

Dead runtime code was removed with the scenes: the skinned/animation loader and
shaders, the weapon view model and projectiles, the vehicle controls, the static
no-uv draw mode, file mesh loading and the spot-light path. The renderer now has
two draw modes only: static + uv and forward water.

The scene list is: **menu**, **physics**, **render**, **ocean**, **shallow**.
The game starts in the menu backdrop. The menu has **Scenes**, **Options** and
**Exit**; scenes can be entered from the menu or from the pause menu.

ESC no longer quits. While playing it opens a pause overlay with **Resume**,
**Options**, **Scenes**, **Main Menu** and **Exit**. The paused game keeps its
last rendered frame while simulation and camera input stop. Options currently
cover field of view, vsync and master volume.

`GameWindow` owns input and timing only; `ux.h` owns the menu/pause state and
scene selection. The scene manager remains the only thing that loads or unloads
a scene. Closing the window or choosing Exit shuts the engine down cleanly.

## Physics scene

`scene_physics` is a bench of labelled stations, one per Bullet feature. The
stations are inspectable, and a compact unit-test runner checks them
automatically after 2.6 s. The HUD shows `unit tests N/15`, and the Scenes tab
has a pass/fail table plus a `rerun tests` button.

Stations:

- primitive shapes (box, sphere, capsule, cylinder, cone)
- a stacked tower
- dominoes driven by a rolling ball
- a linked ragdoll
- a limited hinge
- a cone-twist bag
- a five-link chain
- a see-saw
- a continuous hinge motor
- Newton's cradle
- an inclined ramp
- CCD on/off spheres at a thin wall
- a motorised door
- a linear slider constraint
- a fixed constraint
- a wind-driven flagpole with a soft flag
- a low suspended soft cloth with a dropped ball
- a walk-through soft curtain in a doorway

The physics layer also exposes point-to-point, slider, fixed, hinge, cone-twist
and hinge2 constraints, raycasts, impulses, continuous forces and trigger
sensing. Motorised bodies are kept awake explicitly because Bullet motors do
not wake sleeping bodies themselves.

The flag, cloth and curtain are **real Bullet `btSoftBody` patches** in a
`btSoftRigidDynamicsWorld` with a `btSoftBodyRigidBodyCollisionConfiguration`
and `btDefaultSoftBodySolver`. Only the flag's hoist column is pinned, so the
cloth can billow; wind has a small lift component and no lateral component, so
it flutters without swinging around the pole. The cloth is pinned around its
perimeter, sits low and is deformed by a heavy rigid sphere. The curtain is
pinned along its top edge in a doorway and can be pushed through. Soft-rigid
collision uses the SDF path because `CreatePatch` has no clusters. Soft-body
materials are marked with `mat_b.b = 2` and the geometry shader draws a
distance-faded warp/weft/twill weave, slub and fibre variation, a herringbone
tonal stripe, folded hem, stitched seam and a darker back-face lining from the
grid UVs instead of sampling a texture. Their faces are rebuilt into a dynamic static-buffer mesh
each frame, with normals accumulated from the current triangle positions. The
deferred pass is double-sided so soft sheets are visible from either side.

The automated tests cover: shapes settle, stack stable, dominoes topple, hinge
swings, cone hang, chain links, see-saw tilt, motor turns, CCD stops, cradle
transfer, ramp roll, flag deformation, cloth deflection, slider travel and fixed
joint integrity.

## Render scene

`scene_render` remains the material lab: a studio preset, a neutral stage, a
metal/roughness matrix, ten named material spheres, local-light stress volumes,
a texture-vs-flat comparison and a live hero material edited in the Scenes tab.
It is a test bed for the deferred g-buffer and lighting, not gameplay content.

The studio preset uses a brighter sun and ambient fill than the old dark set,
and the lab surfaces sit at mid-grey so the material spheres and metal/roughness
matrix stay readable. Four coloured point lights sit just above the material
row; their bulbs are real props, so the lights have a visible source. Light
debug wires are deliberately sparse: a small source marker, a drop line and the
circle where the reach meets the stage top, drawn in the light's own colour. The
Debug tab and the Render scene's Scenes controls can toggle local lighting and
the wires.

## Water scenes

There are two water paths:

- **ocean**: a CPU FFT patch tiled into open sea, with a dynamic foam field and
  a short-chop detail normal map. It uses the forward water pass with
  refraction, absorption, sky reflection, sun specular and crest scattering.
- **shallow**: a small finite-difference patch inside a stone basin, showing
  drops, reflection and damping at a much smaller scale.

The forward water pass is double-sided: the sea is one layer of triangles, so
from below and wherever a choppy crest folds over itself the back faces are
still water. Culling them punched holes that showed the sky disk or the dark
seabed through the surface. Surface normals are clamped above the horizon, so a
steep facet can never reflect the dark ground hemisphere into the sea, and the
reflection environment contains no hard sun/moon disk: the disk belongs to the
sky pass, while water gets a broad scatter plus a GGX glint that widens with
distance. Fog over water always samples the sky hemisphere.

Both water scenes feed the same water uniforms into the renderer and keep their
own Scenes controls.

## Removed systems

Removed from the active code and project files:

- procedural city graph, zoning, occupancy plan and streaming
- procedural building generator and preview
- city and building shaders, city draw modes and mesh entry points
- hydraulic/traffic/pedestrian/signal simulation
- hub, weapons, terrain/world and vehicle scenes
- terrain heightfield world generation

`src/world.h` is archived under `megastudy/terrain-archive/`. The city snapshot,
shaders, diff and notes are under `megastudy/procgen-city/`.

## Conventions

- Plain structs, free functions, tables and explicit switches. No inheritance or virtual functions.
- Scenes own their content and are rebuilt from scratch on a switch.
- Every scene has its own HUD and its own debug controls.
- Comments are short and standalone; they describe what the code does now.
