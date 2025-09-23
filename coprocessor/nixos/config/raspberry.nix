{ lib, ... }:
{
    # Reduce image size
    documentation.enable = false;

    # TEMP: override kernel params to remove serial console
    boot.kernelParams = lib.mkForce [
        "g_ether.dev_addr=32:98:33:00:00:00"
        "g_ether.host_addr=32:98:33:00:00:01"
        # "console=serial0,115200n8"
        "console=tty1"
        "nohibernate"
        "loglevel=7"
        "lsm=landlock,yama,bpf"
    ];

    # Configure board setup
    boot = {
        kernelModules = [ "i2c_dev" ];
    };

    users.users.pi.extraGroups = [
        "i2c"
        "gpio"
        "dialout"
    ];

    hardware.raspberry-pi = {
        config = {
            all = {
                base-dt-params = {
                    # Enable i2c (used for OTOS connection)
                    i2c_arm = {
                        enable = true;
                        value = "on";
                    };
                };
                options = {
                    # Enable uart (used for brain communication)
                    enable_uart = {
                        enable = true;
                        value = true;
                    };
                };
                dt-overlays = {
                    # Disable bluetooth, as it isn't necessary
                    disable-bt = {
                        enable = true;
                        params = { };
                    };
                };
            };
        };

        # https://github.com/nvmd/nixos-raspberrypi/issues/98
        extra-config = ''
            [all]
            # Drive RE & DE early in boot to avoid sending boot messages to the brain
            gpio=17=op,dh
            gpio=18=op,dl
        '';
    };
}
