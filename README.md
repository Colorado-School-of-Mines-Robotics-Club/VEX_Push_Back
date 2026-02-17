# Push Back

## Development environment setup

A nix flake is provided with a `devShell` output providing all the necessary
tools. If you have nix installed, you can use that. If you don't, all the
necessary tools are listed below:

1. `rustup`
    - Follow the instructions at https://www.rust-lang.org/tools/install
    - This project requires nightly and has a local `rust-toolchain.toml` defining the specific toolchain version.
2. `cargo-v5`
    - This is the  CLI for building and uploading to the brain
    - `cargo install --git https://github.com/vexide/cargo-v5`
3. `mpremote`
    - This is used for interacting with micropython, in our case on our RPI Pico coprocessor
    - It can be installed from pip (or a system package manager)

## Project layout

As of writing, all the code that goes into the robot is as described below:
| Path | Description |
| - | - |
| `bots/*` | Individual rust projects for the bots, containing robot-specific code such as port assignment, autons, and any top-level setup |
| `common/*` | Rust projects used by any/multiple bots, such as common subsystem logic and display UI |
| `misc/*` | Miscellaneous rust crates, some random experiments or WIP code that might be used in the future |
| `coprocessor/pico/` | The micropython code for the RPI Pico |
| `coprocessor/brain/` | The rust crate that handles communication between the brain and the coprocessor |
