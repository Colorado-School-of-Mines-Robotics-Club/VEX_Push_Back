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

## Project layout

As of writing, all the code that goes into the robot is as described below:
| Path | Description |
| - | - |
| `src/*` | The main robot code entrypoint, contains most robot-specific code |
| `coprocessor/copro.py` | The python script that runs on an RPI Zero 2W connected to the brain |
| `coprocessor/nixos/*` | The NixOS configuration for the RPI Zero 2W |
| `coprocessor/brain/*` | The rust crate that handles communication between the brain and the RPI Zero 2W |
| `display/src/*` | A rust crate providing UI code that runs on the brain, including an auton selector |
| `display/simulator/*` | An isolated rust crate that can be run on a host device and simulates the brain display |
