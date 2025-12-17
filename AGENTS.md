# AGENTS.md — Universal Arduino conventions

## Purpose
Universal rules for Arduino repositories in this workspace.
Keep this file short and actionable. Put project-specific details in a nested `AGENTS.md`.

## Language policy (hard rule)
- All identifiers are English-only.
- All comments are English-only.
- Chat/discussion may be Dutch, but code + comments remain English.

## Scope
- **Focus:** Code changes only (Arduino/C++).
- **Out of scope:** Uploading/flashing and serial monitoring.
- **Hardware constraints:** Do not change board/FQBN, clock assumptions, pin assignments, or wiring assumptions without explicitly calling it out and asking first.

## Definition of done
- Use Arduino CLI compile-only to verify “compiles cleanly” (no upload/flash), unless the user explicitly says the repo is WIP/non-compiling.
  - Compile with warnings enabled; don’t add new warnings in **project code** (upstream/core warnings may exist).
- For behavior changes: describe expected runtime behavior and any new assumptions.

## Readability (high priority)
- **Baseline:** Keep code understandable for intermediate Arduino programmers.
- **Simplicity:** Prefer simple, explicit, easy-to-follow code over cleverness.
- **Clarity:** Avoid overly compact expressions; prefer clarity.
- **Target Audience:**
  - Main/example sketches (`.ino`): Beginner-friendly / Cockpit view.
  - Libraries/Modules (`.h`/`.cpp`): Intermediate-level.
- **Naming:** Use descriptive names; avoid cryptic abbreviations. Only use well-known acronyms (e.g., PID, CAN, IMU).
- **No Tricks:** Avoid heavy templates, metaprogramming, dense operator tricks, or implicit conversions.
- **Comments:** Add comments when intent, units, assumptions, or hardware constraints are not obvious from the code.

## Sketch structure (Arduino rule)
- A sketch is a folder.
- The primary `.ino` file must match the sketch root folder name.
- Additional `.h`, `.cpp` files may live in the same sketch folder.

## Formatting
- Indentation: 2 spaces (avoid tabs in committed code).
- Braces: K&R / 1TBS style.
- Always use braces for control statements, even single-line bodies.

## Naming conventions
- **Functions + variables:** lowerCamelCase
- **Types (struct/class/enum):** PascalCase
- **Acronyms:** inside identifiers
  - **2-letter:**  UPPERCASE (e.g., IOStream, runUI).
  - **3+ letter:** capitalize only the first letter (e.g., HttpServer, parseJson).
- **Constants/macros:** ALL_CAPS (e.g., MAX_RETRIES, SERIAL_BAUD)
- **Files (non-.ino):** In libraries, the main public header/source match the library name; otherwise keep naming consistent within the repo.

## Error handling (hard rule)
- Only fallible functions use this pattern.
- Return `ClassName::ERROR` defined as: `enum ERROR : uint8_t { ERROR_OK = 0, ... };`
- Return data via **out-parameters**.
- Callers check explicitly (no `auto`):
  - `ClassName::ERROR rc = obj.readValue(addr, outValue);`
  - **Caller handling:** `if (rc == ClassName::ERROR_OK) { ... }`
  - **Internal propagation:** `if (rc != ERROR_OK) return rc;`
- **Prohibited:** No `String` status returns, no exceptions, no global error macros, no debug text in return values.

## Data modeling
- **Config vs State:** Separate configuration from runtime state.
  - Config examples: gains/limits/calibration.
  - State examples: setpoints/measurements/outputs/mode flags.
- **Scope:** Prefer smallest reasonable scope.
  - Prefer scope order: local > member > file-static > global.
  - **Globals (allowed, but intentional):** Globals are acceptable for shared sketch state or hardware singletons, but avoid “random globals” and document intent when not obvious.
- **Encapsulation:** In libraries, use classes/structs. In sketches, prefer simple structs over complex classes unless behavior is repeated.

## Timing / scheduling / Lifecycle
- **Initialization:** Don’t hide hardware init or Arduino API calls in constructors or global initializers. Use `setup()` or a `begin()` method.
- **Non-blocking:** Prefer `millis()`-based scheduling. Avoid indefinite blocking waits/loops.
- **delay() exception:** Allow a temporary `delay()` only as a documented bring-up placeholder; remove it once real timing/work is implemented.
- **Overflow safety:** Use `uint32_t` for `millis()`/`micros()` timestamps with overflow-safe delta checks.
  - **Overflow-safe delta example:** `if ((uint32_t)(now - last) >= intervalMs) { ... }`
- **Responsiveness:** If waiting for hardware (e.g. WiFi, CAN), keep the system responsive (non-blocking or retry). Infinite waits only when explicitly intended + documented.

## Control-flow & Architecture
- **Orchestrator (.ino):** The `.ino` file is a readable cockpit. It orchestrates high-level logic using `setup()` and `loop()`.
- **Subsystems (Modules):** Organize logical units (e.g., "MotorControl") as modules exposing `XxxSetup()` and `XxxLoop()` functions.
- **Components (Classes):** Only create classes for repeatable, stateful building blocks (e.g., Button, PIDController) to avoid duplication.
  - **Building blocks:** Put reusable building blocks in dedicated headers (and `.cpp` when it’s a library).
- **Guard Clauses:** Prefer `if (!enabled) return;` over deep nesting.
- **Function Length:** Aim for short functions (≤ ~60 lines).
  - Extract helpers when scanning the function becomes difficult (don’t wait for a hard line limit).
- **Mode Matrices:** Prefer `enum + switch` for explicit mode/state machine handling.
- **Events first:** Separate input/event scanning from actions; collect events, then handle in one switch/dispatch.

## Memory / performance & tooling hygiene
- **No Dynamic Allocation:** Avoid `new`/`delete`/`malloc`/`free` after initialization. Avoid heavy `String` churn.
- **Buffers:** Prefer fixed-size buffers and arrays.
- **Logging:** Keep logging optional and lightweight (verbosity flag or compile-time macro).
- **Pointers:** Prefer references or (ptr + length) pairs. Use raw pointers only when required by an API or for optional outputs; check for `nullptr` before dereferencing; avoid pointer-to-pointer; don’t hide dereferences in macros/typedefs.
  - **Function pointers:** Use only when an API requires callbacks; otherwise prefer normal functions/objects.
- **Preprocessor:** Use for includes/guards and simple macros (constants/logging); avoid complex macros.
- **Conditional Compilation:** Keep `#if`/`#ifdef` rare—only for board/core/library differences or explicit build flags; prefer `constexpr`, `enum`, or normal C++ otherwise.

## Headers vs .cpp
- **Personal sketches:** Header-only definitions are acceptable for iteration speed.
  - If you hit multiple-definition/ODR issues: prefer `static`/`inline`, or keep the definition in a single translation unit.
- **Libraries:** Declarations in `.h`, definitions in `.cpp`.
- Do not refactor sketch → library layout unless explicitly requested.

## Agent working style
- Make one coherent change-set at a time.
- Do not rename public-facing identifiers unless asked or clearly necessary.
- **Preference Choices:** When a choice is preference-based, present A/B options with a **one-line trade-off** and **ask the user**.
- **Ask, don't guess (broader):** If uncertain about expected runtime behavior, existing repo conventions, API assumptions, or hardware constraints, stop and ask.
