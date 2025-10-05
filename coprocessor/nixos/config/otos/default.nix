{ pkgs, self, ... }:
{
    environment.systemPackages = [ self.packages.${pkgs.system}.pythonWithLibs ];

    # systemd.services.otos-sensor = {
    #     description = "otos sensor";
    #     wantedBy = [ "multi-user.target" ];

    #     environment = {
    #         PYTHONUNBUFFERED = "true"; # Fix python stdout/stderr with syslog
    #     };

    #     serviceConfig = {
    #         User = "root";
    #         Group = "root";

    #         ExecStart = "${lib.getExe self.packages.${system}.otos-script}";
    #     };
    # };
}
