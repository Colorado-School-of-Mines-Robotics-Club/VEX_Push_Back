{
    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
        flake-utils.url = "github:numtide/flake-utils";
        rust-overlay = {
            url = "github:oxalica/rust-overlay";
            inputs.nixpkgs.follows = "nixpkgs";
        };
        treefmt-nix = {
            url = "github:numtide/treefmt-nix";
            inputs.nixpkgs.follows = "nixpkgs";
        };
    };

    outputs =
        inputs@{
            self,
            nixpkgs,
            flake-utils,
            rust-overlay,
            treefmt-nix,
            ...
        }:
        flake-utils.lib.eachDefaultSystem (
            system:
            let
                # Import nixpkgs
                pkgs = nixpkgs.legacyPackages.${system};
                inherit (pkgs) lib;
                # Setup rust toolchain
                rust-bin = rust-overlay.lib.mkRustBin { } pkgs;
                rust' = (rust-bin.fromRustupToolchainFile ./rust-toolchain.toml);
                # Setup formatter
                treefmtCfg = lib.modules.importApply ./treefmt.nix { inherit rust'; };
                treefmt = treefmt-nix.lib.evalModule pkgs treefmtCfg;
                # Make winit work on NixOS
                libPath =
                    with pkgs;
                    lib.makeLibraryPath [
                        libGL
                        libxkbcommon
                        wayland
                    ];
            in
            {
                # Provide a development environment with rust, cargo-v5, and the formatter
                devShells.default = pkgs.mkShell {
                    packages = [
                        rust'
                        self.formatter.${system}
                    ]
                    ++ (with pkgs; [
                        evcxr
                        SDL2
                        mpremote
                        micropython
                        slint-viewer
                        pioasm
                        cargo-v5
                        python3
                    ]);

                    LD_LIBRARY_PATH = libPath;
                };

                formatter = treefmt.config.build.wrapper;
            }
        );
}
