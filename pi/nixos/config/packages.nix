{ pkgs, ... }:
{
    environment.systemPackages = with pkgs; [
        neovim
        btop
        i2c-tools
    ];
}
