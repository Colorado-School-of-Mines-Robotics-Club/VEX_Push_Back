{
    config,
    pkgs,
    lib,
    piNumber,
    ...
}:
{
    networking = {
        hostName = "mines-rpi0-${builtins.toString piNumber}";

        firewall.enable = false; # TODO Maybe?

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

    # Enable tailscale for easy connection on any network
    services.tailscale = {
        enable = true;
        useRoutingFeatures = "both";
        openFirewall = true;

        authKeyFile = "${config.sops.secrets."tailscale/preauth_key".path}";

        extraUpFlags = [
            "--reset"
            "--ssh"
            "--advertise-tags=tag:rpi"
            "--login-server=https://ts.myriation.xyz"
            "--operator=pi"
        ];
    };
    services.networkd-dispatcher = {
        enable = true;
        rules = {
            "50-tailscale-optimizations" = {
                onState = [ "routable" ];
                script = ''
                    ${lib.getExe pkgs.ethtool} -K "$IFACE" rx-udp-gro-forwarding on rx-gro-list off
                '';
            };
        };
    };
}
