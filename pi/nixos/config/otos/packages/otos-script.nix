{
    python3,
    python3Packages,
    qwiic-otos,
    ...
}:
python3Packages.buildPythonApplication {
    pname = "otos-script";
    version = "0.0.1";
    pyproject = false;
    dontUnpack = true;

    propagatedBuildInputs = [
        (python3.withPackages (
            pythonPackages:
            with pythonPackages;
            [
                cobs
                pyserial
            ]
            ++ [ qwiic-otos ]
        ))
    ];

    installPhase = ''
        install -Dm755 "${../../../otos-script.py}" "$out/bin/otos-script"
    '';

    meta = {
        mainProgram = "otos-script";
    };
}
