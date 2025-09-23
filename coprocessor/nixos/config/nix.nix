{ pkgs, ... }:
{
    nix = {
        settings = {
            extra-experimental-features = [
                "nix-command"
                "flakes"
            ];

            # Enable binary caches on system
            substituters = [
                "https://nixos-raspberrypi.cachix.org"
                "https://numtide.cachix.org"
            ];
            trusted-substituters = [
                "https://nixos-raspberrypi.cachix.org"
                "https://numtide.cachix.org"
            ];
            trusted-public-keys = [
                "nixos-raspberrypi.cachix.org-1:4iMO9LXa8BqhU+Rpg6LQKiGa2lsNh/j2oiYLNOQ5sPI="
                "numtide.cachix.org-1:2ps1kLBUWjxIneOy1Ik6cQjb41X0iXVXeHigGmycPPE="
            ];

            auto-optimise-store = true;
        };
    };
}
