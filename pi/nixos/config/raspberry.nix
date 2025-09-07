{
    boot = {
        kernelModules = [ "i2c_dev" ]; # TODO: Test if necessary
    };

    hardware.enableAllFirmware = true; # TODO: Test if necessary

    hardware.raspberry-pi.config = {
        all = {
            base-dt-params = {
                # Enable i2c (used for OTOS connection)
                i2c_arm = {
                    enable = true;
                    value = "on";
                };
            };
        };
    };
}
