{
    inputs = {
        nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
        flake-utils.url = "github:numtide/flake-utils";
        rust-overlay = {
            url = "github:oxalica/rust-overlay";
            inputs.nixpkgs.follows = "nixpkgs";
        };
        cargo-v5 = {
            url = "github:vexide/cargo-v5";
            inputs = {
                nixpkgs.follows = "nixpkgs";
                rust-overlay.follows = "rust-overlay";
            };
        };
        treefmt-nix = {
            url = "github:numtide/treefmt-nix";
            inputs.nixpkgs.follows = "nixpkgs";
        };
        nixos-raspberrypi.url = "github:nvmd/nixos-raspberrypi/main";
    };

    outputs =
        inputs@{
            self,
            nixpkgs,
            flake-utils,
            rust-overlay,
            cargo-v5,
            treefmt-nix,
            nixos-raspberrypi,
            ...
        }:
        {
            nixosConfigurations =
                let
                    mkPi =
                        piNumber:
                        nixos-raspberrypi.lib.nixosSystem {
                            specialArgs = {
                                inherit inputs self piNumber;
                                inherit (inputs) nixos-raspberrypi;
                            };
                            modules = [ ./coprocessor/nixos ];
                        };
                in
                {
                    rpi1 = mkPi 1;
                    rpi2 = mkPi 2;
                };
        }
        // flake-utils.lib.eachDefaultSystem (
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
                # Build extra python packages
                mkPythonPackages = ps: rec {
                    qwiic-i2c = pkgs.callPackage ./coprocessor/nixos/config/otos/packages/qwiic-i2c.nix {
                        python3Packages = ps;
                    };
                    qwiic-otos = pkgs.callPackage ./coprocessor/nixos/config/otos/packages/qwiic-otos.nix {
                        inherit qwiic-i2c;
                        python3Packages = ps;
                    };
                };
            in
            {
                # Provide a development environment with rust, cargo-v5, and the formatter
                devShells.default = pkgs.mkShell {
                    packages = [
                        rust'
                        cargo-v5.packages.${system}.cargo-v5-full
                        self.formatter.${system}
                    ]
                    ++ (with pkgs; [
                        (python313.withPackages (
                            pythonPackages: with pythonPackages; [
                                cobs
                                crc
                            ]
                        ))
                        evcxr
                        SDL2
                    ]);
                };

                formatter = treefmt.config.build.wrapper;

                packages.pythonWithLibs = pkgs.python313.withPackages (
                    ps:
                    (with ps; [
                        cobs # For packet delimited serial
                        pyserial # For serial communication
                        crc # For message checksums
                        smbus2 # For i2c communication
                        rpi-gpio # For controlling GPIO pins
                    ])
                    ++ (with (mkPythonPackages ps); [
                        # Add both of the vendor-specific i2c/otos libs
                        qwiic-i2c
                        qwiic-otos
                    ])
                );
            }
        );

    nixConfig = {
        extra-substituters = [ "https://nixos-raspberrypi.cachix.org" ];
        extra-trusted-public-keys = [
            "nixos-raspberrypi.cachix.org-1:4iMO9LXa8BqhU+Rpg6LQKiGa2lsNh/j2oiYLNOQ5sPI="
        ];
    };
}
