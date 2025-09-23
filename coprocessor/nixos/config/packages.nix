{ pkgs, ... }:
{
    environment.systemPackages = with pkgs; [
        neovim
        btop
        zellij
        i2c-tools
    ];
}
