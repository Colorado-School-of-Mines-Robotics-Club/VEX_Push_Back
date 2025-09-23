{ lib, ... }:
{
    # Normal setup is handled by nixos-raspberrypi, we just want to reconfigure the network
    networking.interfaces.usb0 = lib.mkForce { };

    networking = {
        firewall.trustedInterfaces = [ "usb0" ];
        useNetworkd = true;
    };
    systemd.network = {
        networks."10-usb-gadget" = {
            matchConfig = {
                Name = "usb0";
            };

            linkConfig.RequiredForOnline = "no";

            networkConfig = {
                # Attempt to configure DHCP over the link. This allows piggybacking off
                # of a connected computer's internet, which is usefull to overcome wireless
                # slowness (and one of our Pis appears to have broken its wireless somehow)
                DHCP = "yes";
                IPv6AcceptRA = "yes";

                # Configure link-local addresses to ensure host-pi communication is always
                # functional, even if DHCP/RA fails
                LinkLocalAddressing = "ipv6";

                # Configure static-ish link local addresses for ease of use in case mDNS fails
                IPv4LLStartAddress = "169.254.25.0";
                IPv6LinkLocalAddressGenerationMode = "eui64";

                # Enable mDNS to make host-pi communication easy
                MulticastDNS = true;

                IgnoreCarrierLoss = "3s";
            };
        };
    };
}
