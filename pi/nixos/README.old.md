# Raspberry PI

Our bots use a Raspberry PI Zero W 2 to interface with the brain and an optical tracking sensor on the bottom of the robot. For ease of use and reproducibility, both are running NixOS with the system configuration in the current directory.

## Setup instructions (requires `just`)

1. Build the SD card image using `just build-sd [pi number]`
2. Plug in an SD card using a slot or an USB adapter
3. Write the SD card image to the card using `just write-sd <device> [pi number] [image path]`
4. Put SD card into pi and plug it into computer using micro USB cord
	- NOTE: In order to get a proper ssh connection, you MUST use the left micro USB port (labeled "USB"). The other port only supports providing power to the pi.
5. On your laptop, using NetworkManager (or another method), configure the new ethernet interface with a static IP with the following configuration
	- Static IPv4 address: 10.0.0.100
	- Subnet mask: 255.255.255.0
	- Gateway: 0.0.0.0
	- IPv6 disabled
6. Run the initial setup for the pi with `just setup-pi [number] [sops-key-path]`
	- Make sure you have the proper sops private key that `./secrets.yaml` is encrypted with (see the **sops-nix** section)
7. Either press `y` when asked if you want to reboot, or SSH manually to `pi@10.0.0.2` and do any extra changes before rebooting. **Rebooting will disable ssh access over micro USB!**
8. The pi will now be set up with the USB port properly configured for v5 brain communication, i2c setup for OTOS communication, and tailscale login for easy modification.
	- As the pi should now be logged into tailscale, ssh over tailscale from any device should work without any password or ssh key login.
	- Just use name of the pi (`mines-rpi0-<number>`) as the IP (or the tailscale IP, shown with `tailscale status`)

## NixOS Config

The main entrypoint for the system configuration is `./default.nix`.

The config uses a couple of helpful external flakes/modules:
- [`github:nix-community/nixos-facter[-modules]`](https://github.com/nix-community/nixos-facter), to automatically handle hardware-specific NixOS configuration
- [`github:Mic92/sops-nix`](https://github.com/Mic92/sops-nix), to securely store secrets that need to be placed on the pi (such as a tailscale pre-auth key)
- [`github:nvmd/nixos-raspberrypi`](https://github.com/nvmd/nixos-raspberrypi), for packaging the correct kernel, managing the raspberry pi bootloader, and configuring the firmware

The configuration includes:

- Normal NixOS boilerplate (users, ssh, locale, a few installed packages, etc)
- Hardcoded MAC addresses on the Wi-Fi chip and USB gadget ethernet
- Wi-Fi setup, autoconnecting to a few networks
- Tailscale setup, which
	- Connects to a private headscale server, but this is easy to change/configure
	- Advertises `tag:rpi`, so ensure that tag exists and has proper ACL permissions
	- Enables `tailscale ssh` to allow easy connections over tailscale
- `i2c` configuration for the raspberry pi firmware, used for connection to the OTOS sensor
- A systemd service that runs `./otos-script.py` on boot
	- This includes packaging for a couple of SparkFun python libraries that aren't packaged by nixpkgs
- Only in the initial configuration used to build the SD image: USB OTG gadget ethernet enabled to setup over USB initially

### sops-nix

TODO
