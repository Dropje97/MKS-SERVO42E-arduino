# AGENTS.md — Universal Arduino conventions

## Purpose
Universal rules for Arduino repositories in this workspace.
Keep this file short and actionable. Put project-specific details in a nested `AGENTS.md`.

## Language policy (hard rule)
- All identifiers are English-only.
- All comments are English-only.
- Chat/discussion may be Dutch, but code + comments remain English.

## Scope
- Focus: code changes only (Arduino/C++).
- Out of scope: uploading/flashing and serial monitoring (handled by the user in Arduino IDE GUI).
- Do not change board/FQBN, clock assumptions, pin assignments, wiring assumptions, or voltage levels without explicitly calling it out and asking first.

## Definition of done
- The sketch compiles cleanly, unless the user explicitly says the repo is WIP/non-compiling.
- For behavior changes: describe expected runtime behavior and any new assumptions.

## Readability (high priority)
- Prefer simple, explicit, easy-to-follow code over cleverness.
- Keep code understandable for intermediate Arduino programmers; main sketches (.ino) should preferably be beginner-friendly, while libraries don’t need to be beginner-approachable but should stay within intermediate-level complexity.
- Use descriptive names; avoid cryptic abbreviations. Only use well-known acronyms (e.g., `PID`, `CAN`, `IMU`) and keep them UPPERCASE.
- Avoid “clever C++” that hides intent (heavy templates, metaprogramming, dense operator tricks).
- Add English comments when intent, units, assumptions, or hardware constraints are not obvious from the code.
- Prefer small functions with clear responsibilities; avoid overly compact expressions.

## Sketch structure (Arduino rule)
- A sketch is a folder.
- The primary `.ino` file must match the sketch root folder name.
- Additional `.ino`, `.h`, `.cpp` files may live in the same sketch folder.

## Formatting (Arduino-default oriented)
- Indentation: 4 spaces (avoid tabs in committed code).
- Braces: K&R / 1TBS style.
- Always use braces for control statements, even single-line bodies.

## Naming conventions (Arduino-friendly)
- Functions + variables: `lowerCamelCase`
- Types (struct/class/enum): `PascalCase`
- Acronyms: UPPERCASE inside identifiers (e.g., `PIDConfig`, `CANFrame`, `IMUData`)
- Constants/macros: `ALL_CAPS` (e.g., `MAX_RETRIES`, `SERIAL_BAUD`)
- File naming (non-.ino): `lower_snake_case.h/.cpp` unless a project requires otherwise.

## Data modeling
- Separate config from runtime state:
  - Config structs: tunables / persisted settings (gains, limits, calibration).
  - State structs: current setpoints, measurements, outputs, mode flags.
- Globals are acceptable for sketches (single-controller style). Avoid abstraction for its own sake.

## Timing / scheduling
- Prefer non-blocking timing (“BlinkWithoutDelay” style) over `delay()`.
- Use `uint32_t` for `millis()` / `micros()` timestamps.
- Use overflow-safe delta checks:
  - `if ((uint32_t)(now - last) >= intervalMs) { ... }`
- If a temporary `delay()` is used during bring-up, leave an English comment explaining why and remove it later.

## Control-flow style
- Prefer guard clauses over deep nesting: `if (!enabled) return;`
- Prefer `enum` + `switch` for explicit mode matrices.
- Keep functions short; extract helpers once scanning becomes difficult.

## Memory / performance (Arduino realities)
- Avoid dynamic allocation in tight loops (and heavy `String` churn) unless explicitly justified.
- Prefer fixed-size buffers.
- Keep logging optional and lightweight (verbosity flag or compile-time macro).

## Headers vs .cpp (context-dependent)
- Personal sketches: header-only definitions are acceptable for iteration speed.
- Reusable libraries/shared code: headers are declarative; definitions in `.cpp`.
- Do not refactor sketch → library layout unless explicitly requested.

## Agent working style
- Make one coherent change-set at a time.
- Do not rename public-facing identifiers unless asked or clearly necessary.
- When a choice is preference-based, present A/B with a one-line trade-off and ask the user.
- When uncertain about hardware, expectations, or style details: ask instead of guessing.
