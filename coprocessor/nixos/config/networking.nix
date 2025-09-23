{ piNumber, ... }:
{
    networking = {
        hostName = "ordgr-rpi0-${builtins.toString piNumber}";

        firewall = {
            enable = true;
            allowedTCPPorts = [
                # SSH
                22
            ];
            allowedUDPPorts = [
                # mDNS
                5353
            ];
        };
        nftables.enable = true;

        # Configure wifi
        wireless = {
            enable = true;
            userControlled.enable = true;
            networks = {
                # Mines unauthenticated network
                "Mines-Legacy" = {
                    priority = 100;
                    authProtocols = [ "NONE" ];
                };
                # Tyler hotspot
                "typixel" = {
                    priority = 50;
                    psk = "hhhhhhhh";
                };
            };
        };

        # networking is managed by systemd-networkd and wpa_supplicant, disable everything else
        networkmanager.enable = false;
        useDHCP = false;
    };

    # Use systemd-networkd for DHCP on the Wi-Fi interface
    systemd.network = {
        enable = true;

        networks."10-wireless" = {
            matchConfig = {
                Name = "wlan0";
            };

            linkConfig.RequiredForOnline = "yes";

            networkConfig = {
                DHCP = "yes";
                IPv6AcceptRA = "yes";

                IgnoreCarrierLoss = "3s";
            };
        };
    };
}
