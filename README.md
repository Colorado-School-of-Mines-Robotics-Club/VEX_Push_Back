# ORDGR Vex Push Back robot code

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
| `misc/*` | Primarily a dumping ground for random ideas or WIP code, such as more advanced control theory or vexide utilities. None of this code is finished or tested, hence it not actually being used in our competition code. At some point, if finished, the code in here may get refactored out into proper crates. |
| `coprocessor/pico/` | The micropython code for the RPI Pico 2. Supports reading OTOS values and controlling WS2812b LEDs |
| `coprocessor/brain/` | The rust crate that handles communication between the brain and the coprocessor |
