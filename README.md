# drone-planner-rust

Native Rust rewrite of the Drone Planner (Robolink CoDrone EDU flight planner,
simulator, and code generator). A portable desktop application that runs on
low-end school Windows laptops WITHOUT admin rights.

Target machine: botched school Win11, 4th-gen i5, 4GB RAM, Intel integrated GPU.
Renderer: eframe + egui + GLOW (OpenGL 3.3-class). No wgpu, no Vulkan, no WebView.

## Status

- Scaffold complete: workspace, crate skeletons, fixture loader, DP_SMOKE gate
- Engine core (aero, simulator, commands, obstacles, codegen, plan I/O): next
- Scene + UI (viewport, plan tree, palette, telemetry, obstacles panel): queued
- Portable Windows exe packaging: queued

## Layout

```
crates/planner-core     engine library: no GUI deps, fixture-tested
  src/aero.rs           point-mass flight physics
  src/commands.rs       58 command types, params schema, defaults
  src/sim/              command interpreter (drive, moves, shapes, runtime, controlflow)
  src/obstacles.rs      obstacle set, AABB collision, boundary filter, importers
  src/codegen.rs        CoDrone EDU Python emitter + script parser
  src/planio.rs         plan / obstacle JSON load-save
  src/golden.rs         frozen fixture types (serde)
crates/planner-app      desktop binary (eframe glow, rfd file dialogs)
  DP_SMOKE=1            headless smoke gate (no window, prints and exits 0)
tests/golden/golden.json  frozen outputs captured from the reference web app (11
                          plans, 3-drone swarm, obstacle set, collision plan)
tests/parity.rs         fixture load + command-type identity tests
```

## Build and run

Linux dev:

```
cargo build --workspace
cargo test --workspace
DP_SMOKE=1 cargo run -p planner-app   # headless smoke, no window
cargo run -p planner-app              # window
```

Windows target (compile gate; final exe ships with the packaging stage):

```
rustup target add x86_64-pc-windows-msvc
cargo check --target x86_64-pc-windows-msvc
```

The shipped exe is built with a static CRT (`-C target-feature=+crt-static`), needs
no VC redistributable, writes only under `%APPDATA%\drone-planner`, touches no
registry, and starts with zero network. If the machine lacks a GPU driver, drop a
mesa llvmpipe `opengl32.dll` next to the exe (software rendering, no admin needed).

## Fixture parity

`tests/golden/golden.json` is the ground truth: simulator plans, a swarm run, the
obstacle set, and one collision case, captured from the reference web application.
The Rust engine must reproduce positions, telemetry, point counts, durations,
collision records, and byte-identical generated Python within frozen tolerances
(position 1e-2 m, angles 1e-1 deg, energy/battery 1e-2 relative). Simulator quirks
observed in the reference app are frozen as spec, not fixed: collision checks happen
only in the `go` command, the drone drives through a collided obstacle, and the
post-collision position commit is skipped.
