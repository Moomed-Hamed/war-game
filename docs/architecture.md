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
| `loader.h` | Loading and processing assets from disk |
| `audio.h` | Sound effects and music |
| `window.h` | Window, input, timing, g-buffer, audio device |
| `drawer.h` | Draw call preparation and execution |
| `physics.h` | Bullet world, bodies, joints, heightfields, vehicles |
| `player.h` | Capsule movement, view gun, projectiles |
| `terrain.h` | Heightfield generation, band meshes, terrain shaping |
| `water.h` | FFT ocean, foam field, short-chop normals |
| `shallow.h` | Finite-difference shallow water patch |
| `scene.h` | Scene content, shared builders, scene manager |
| `ux.h` | Console, scene selector, debug tools |
| `main.cpp` | Main loop |

---

## Conventions

- Plain structs, free functions, tables and explicit switches. No inheritance or virtual functions.
- Scenes own their content and are rebuilt from scratch on a switch.
- Every scene has its own HUD and its own debug controls.
- Comments are short and standalone; they describe what the code does now.