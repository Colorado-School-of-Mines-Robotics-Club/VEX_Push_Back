{
    users = {
        users =
            let
                keys = [
                    "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIH/2p94CtvFXI/CXLudCu5KYFf8Afes1o3G7T1wLFeGP openpgp:0x0D009703"
                ];
            in
            {
                pi = {
                    hashedPassword = "$y$j9T$Yzt9KnotPzWowtxphQNyQ1$8QHcZUAm8iYj9/EtRG4BXwygtEoJMNGJqJMSIxDQCuD";
                    isNormalUser = true;
                    group = "pi";
                    extraGroups = [ "wheel" ];

                    openssh.authorizedKeys.keys = keys;
                };
                root.openssh.authorizedKeys.keys = keys;
            };
        groups.pi = { };
    };

    services.openssh.enable = true;

    security.sudo.wheelNeedsPassword = false;

    nix.settings.trusted-users = [ "pi" ];
}
