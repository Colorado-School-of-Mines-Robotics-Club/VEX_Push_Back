{ rust' }:
{ ... }:
{
    projectRootFile = "flake.nix";

    programs = {
        rustfmt = {
            enable = true;
            edition = "2024";
        };
        nixfmt = {
            enable = true;
            strict = true;
        };
    };

    settings.formatter.nixfmt.options = [
        "--indent"
        "4"
    ];
}
