# Push Back

## Project setup

A nix flake is provided with a `devShell` output providing all the necessary
tools. If you have nix installed, you can use that. If you don't, all the
necessary tools are listed below:

1. `rustup`
    - Follow the instructions at https://www.rust-lang.org/tools/install
    - This project requires nightly and has a local `rust-toolchain.toml` defining the specific toolchain version.
2. `cargo-v5`
    - This is the  CLI for building and uploading to the brain
    - `cargo install --git https://github.com/vexide/cargo-v5`
