# drone-planner-rust

Native Rust rewrite of the Drone Planner (Robolink CoDrone EDU flight planner,
simulator, and code generator). A portable desktop application that runs on
low-end school Windows laptops WITHOUT admin rights.

## License

MIT. See [LICENSE](LICENSE). Generated Python uses the official Robolink
`codrone_edu` library API; CoDrone EDU is a product of Robolink Corp. This
project is an independent educational tool and is not affiliated with or
endorsed by Robolink.

Target machine: botched school Win11, 4th-gen i5, 4GB RAM, Intel integrated GPU.
Renderer: eframe + egui + GLOW (OpenGL 3.3-class). No wgpu, no Vulkan, no WebView.

## Status

- Engine core (aero, simulator, commands, obstacles, codegen, plan I/O): implemented, golden-parity tested (82 tests)
- Scene + UI (viewport, plan tree, palette, telemetry, obstacles panel): implemented, feature-parity audited against the reference app
- Portable Windows exe build: see BUILD_LOG.md

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

## Windows build

```
rustup target add x86_64-pc-windows-msvc
cargo install cargo-xwin
RUSTFLAGS="-C target-feature=+crt-static" cargo xwin build --release --target x86_64-pc-windows-msvc
```

The exact command and import-table verification are recorded in BUILD_LOG.md.

## Run on Windows (no admin)

- Copy `drone-planner.exe` (from `target/x86_64-pc-windows-msvc/release/`) anywhere writable: Downloads, Desktop, or a USB stick.
- Double-click it. A window titled "Drone Planner" opens.
- If nothing opens or you get a GL error: drop a mesa llvmpipe `opengl32.dll` next to the exe (software rendering), then start again.
- No admin, no installer, no registry, no network use.
- The app starts writing nothing to disk. Plans, obstacle files, and Python exports go wherever you save them in the file dialogs (default extension `.flight` for plans; JSON/GeoJSON/CSV/OBJ for obstacles).

## Fixture parity

`tests/golden/golden.json` is the ground truth: simulator plans, a swarm run, the
obstacle set, and one collision case, captured from the reference web application.
The Rust engine must reproduce positions, telemetry, point counts, durations,
collision records, and byte-identical generated Python within frozen tolerances
(position 1e-2 m, angles 1e-1 deg, energy/battery 1e-2 relative). Simulator quirks
observed in the reference app are frozen as spec, not fixed: collision checks happen
only in the `go` command, the drone drives through a collided obstacle, and the
post-collision position commit is skipped.
