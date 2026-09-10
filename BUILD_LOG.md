# BUILD LOG - Windows Static-CRT Portable Executable

## Prerequisites

    rustup target add x86_64-pc-windows-msvc
    cargo install cargo-xwin --locked

## Build Command

    RUSTFLAGS="-C target-feature=+crt-static" cargo xwin build --release --target x86_64-pc-windows-msvc

First run downloads the MSVC CRT and Windows SDK (~1.5 min). Subsequent builds
reuse the cached SDK.

## Build Output

    target/x86_64-pc-windows-msvc/release/drone-planner.exe
    Size: 8,151,552 bytes (7.78 MB)
    Format: PE32+ executable for MS Windows 6.00 (console), x86-64, 8 sections

## Import Table Verification

    llvm-readobj --coff-imports target/x86_64-pc-windows-msvc/release/drone-planner.exe

Observed system DLLs (all present are Windows built-ins):

    KERNEL32.dll
    user32.dll
    ole32.dll
    combase.dll
    shell32.dll
    ntdll.dll
    bcryptprimitives.dll
    api-ms-win-core-synch-l1-2-0.dll
    OPENGL32.dll
    GDI32.dll
    ADVAPI32.dll
    dwmapi.dll
    imm32.dll
    uxtheme.dll

Forbidden DLLs (confirming ABSENCE):

    vcruntime140.dll    - NOT PRESENT
    vcruntime140d.dll   - NOT PRESENT
    msvcp140.dll        - NOT PRESENT
    ucrtbase.dll        - NOT PRESENT
    api-ms-win-crt      - NOT PRESENT

## Linux Dev Build

    cargo build --workspace
    cargo test --workspace
    DP_SMOKE=1 timeout 20 cargo run -p planner-app

## Why Static CRT

The MSVC CRT is statically linked (+crt-static) so the exe runs on a stock
Windows 11 machine without requiring a VC++ Redistributable install. No admin
rights needed. Drop the single .exe next to the app and it works.

## Notes

    - cargo-xwin v0.23.1 automatically fetches Windows SDK + CRT on first build
    - Linker LNK4099 warnings about missing PDB files are harmless (debug info
      references, not runtime dependencies)
    - The exe is a console binary; the GUI launches via eframe/egui/OpenGL
