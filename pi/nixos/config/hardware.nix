{ piNumber, ... }:
let
    variants = {
        wifiMacs = {
            "1" = "FE:64:28:75:5A:87";
            "2" = "FE:30:EC:DB:0D:BA";
        };
        etherKernelParams = {
            "1" = [
                "g_ether.dev_addr=f2:5d:47:5e:e5:05"
                "g_ether.host_addr=f2:5d:47:5e:e5:06"
            ];
            "2" = [
                "g_ether.dev_addr=f2:5d:47:5e:e5:07"
                "g_ether.host_addr=f2:5d:47:5e:e5:08"
            ];
        };
    };
in
{
    # Configure static MAC address for ethernet gadget
    boot.kernelParams = variants.etherKernelParams.${builtins.toString piNumber};

    # Configure hardcoded MAC address for Wi-Fi
    hardware.raspberry-pi.config.all.base-dt-params.wifiaddr = {
        enable = true;
        value = variants.wifiMacs.${builtins.toString piNumber};
    };
}
