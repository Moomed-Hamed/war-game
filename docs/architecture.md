# Architecture
---

## One header per file

**Rule:** each header includes *only* the next one down the chain.

Every header in `src/` includes exactly the one below it, so `main.cpp`
effectively pulls the entire engine into one compilation unit:

**Why?**
- To keep things simple & efficient.
- here are no include-guard or link-order issues.

---

## Header Chain

dependences:

| File | Purpose |
|---|---|
| `boilerplate.h` | Includes every external library; game only needs to include this file at the beginning to run |

src:

| File | Purpose |
|---|---|
| `logger.h` | In-game console log|
| `loader.h` | Loading assets from disk + processing them |
| `window.h` | window, input, timing, g-buffer, audio |
| `drawer.h` | draw call prep & excecution |
| `ux.h` | user interface + debug interface |
| `main.cpp` | main game loop |

---

## Conventions

- TODO : figure out later