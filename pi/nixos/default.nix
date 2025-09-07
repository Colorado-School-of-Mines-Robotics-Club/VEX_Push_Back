{ inputs, ... }:
{
    system.stateVersion = "25.05";

    imports =
        let
            rpiModules = inputs.nixos-raspberrypi.nixosModules;
            rpiLib = inputs.nixos-raspberrypi.lib;
        in
        [
            # Base for the Pi Zero
            rpiModules.raspberry-pi-02.base

            # Allow building an SD card from this config
            rpiModules.sd-image
            { sdImage.compressImage = false; }

            # Setup hardware configuration
            inputs.nixos-facter-modules.nixosModules.facter
            { config.facter.reportPath = ./facter.json; }

            # Configure nixpkgs for the RPI
            rpiModules.nixpkgs-rpi
            rpiModules.trusted-nix-caches
            rpiLib.inject-overlays

            ./config
        ];
}
