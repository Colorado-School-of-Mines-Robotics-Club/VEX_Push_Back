{ piNumber, ... }:
let
    variants = {
        wifiMacs = {
            "1" = "FE:64:28:75:5A:87";
            "2" = "FE:30:EC:DB:0D:BA";
        };
    };
in
{
    # Configure static MAC address for ethernet gadget
    boot.kernelParams = [
        "g_ether.dev_addr=32:98:33:00:00:00"
        "g_ether.host_addr=32:98:33:00:00:01"
    ];

    # Configure hardcoded MAC address for Wi-Fi
    hardware.raspberry-pi.config.all.base-dt-params.wifiaddr = {
        enable = true;
        value = variants.wifiMacs.${builtins.toString piNumber};
    };
}
