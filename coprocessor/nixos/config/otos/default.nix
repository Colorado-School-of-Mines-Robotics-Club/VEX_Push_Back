{
    pkgs,
    lib,
    self,
    ...
}:
let
    python3 = self.packages.${pkgs.system}.pythonWithLibs;
in
{
    environment.systemPackages = [ python3 ];

    systemd.services.copro = {
        wantedBy = [ "multi-user.target" ];

        environment = {
            PYTHONUNBUFFERED = "true"; # Fix python stdout/stderr with journald
        };

        serviceConfig = {
            User = "root";
            Group = "root";

            ExecStart = "${lib.getExe python3} /home/pi/copro.py";
        };
    };
}
