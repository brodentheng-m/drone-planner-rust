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

## Install on Windows (no admin, no experience needed)

1. Go to the releases page: https://github.com/brodentheng-m/drone-planner-rust/releases
2. Under the newest release, click `drone-planner.exe` to download it.
3. Find the file in your Downloads folder. You can leave it there, or drag it to your Desktop.
4. Double-click `drone-planner.exe`.
5. A window titled "Drone Planner" opens. That's the whole installation - there isn't one.

Notes:
- Windows may show a blue "Windows protected your PC" box. This appears because the
  app is not code-signed (signing certificates cost money). Click "More info", then
  "Run anyway".
- If the window opens but the 3D view is black or you get an OpenGL error, the
  laptop's graphics driver is too old. Download this file:
  https://github.com/pal1000/mesa-dist-win/releases - pick `mesa-for-...-release-msvc.7z`,
  open it, and copy `x64\opengl32.dll` into the same folder as `drone-planner.exe`.
  That adds software rendering; the app then works on any graphics hardware.
- Nothing is installed, nothing runs at startup, no network access. Deleting the
  exe removes the app completely. Plans you save and Python files you export go
  wherever you pick in the save dialog (plans use the `.flight` extension).

## Build from source (Linux, macOS, or Windows)

Requires Rust: install from https://rustup.rs (one command, user-level, no admin).

```
git clone https://github.com/brodentheng-m/drone-planner-rust
cd drone-planner-rust
cargo run -p planner-app            # run with window
cargo test --workspace              # run the test suite
```

The Windows release exe is cross-compiled from Linux; the exact command and
import-table verification are recorded in BUILD_LOG.md.

On macOS there is no prebuilt download - the app builds itself in about two
minutes. Open Terminal (press Cmd+Space, type "Terminal", press Enter), then
paste this whole block at once and press Enter:

```
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
source "$HOME/.cargo/env"
git clone https://github.com/brodentheng-m/drone-planner-rust
cd drone-planner-rust
cargo run -p planner-app
```

The first paste installs Rust, the rest downloads the source and opens the app.
Every launch after the first: open Terminal, type `cd drone-planner-rust`, press
Enter, type `cargo run -p planner-app`, press Enter. M-series and Intel Macs are
both supported automatically.

## Fixture parity

`tests/golden/golden.json` is the ground truth: simulator plans, a swarm run, the
obstacle set, and one collision case, captured from the reference web application.
The Rust engine must reproduce positions, telemetry, point counts, durations,
collision records, and byte-identical generated Python within frozen tolerances
(position 1e-2 m, angles 1e-1 deg, energy/battery 1e-2 relative). Simulator quirks
observed in the reference app are frozen as spec, not fixed: collision checks happen
only in the `go` command, the drone drives through a collided obstacle, and the
post-collision position commit is skipped.
