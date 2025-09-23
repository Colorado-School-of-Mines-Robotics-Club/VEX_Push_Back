{ nixos-raspberrypi, ... }:
{
    system.stateVersion = "25.05";

    imports =
        let
            rpiModules = nixos-raspberrypi.nixosModules;
        in
        [
            # Base for the Pi Zero
            rpiModules.raspberry-pi-02.base
            rpiModules.usb-gadget-ethernet

            # Allow building an SD card from this config
            rpiModules.sd-image
            { sdImage.compressImage = false; }

            # Configure nixpkgs for the RPI
            rpiModules.nixpkgs-rpi
            rpiModules.trusted-nix-caches
        ]
        ++ [ ./config ];
}
