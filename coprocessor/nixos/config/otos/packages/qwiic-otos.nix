{
    python3Packages,
    qwiic-i2c,
    fetchPypi,
    ...
}:
python3Packages.buildPythonPackage rec {
    pname = "qwiic-otos";
    version = "2.0.0";
    pyproject = true;

    src = fetchPypi {
        inherit version;
        pname = "sparkfun_qwiic_otos";
        hash = "sha256-YzyWw0ix/YyaRfAtVvQ0toNOM+Tsmgym6KnU11S21zQ=";
    };

    build-system = with python3Packages; [
        setuptools
        setuptools-scm
    ];

    dependencies = [ qwiic-i2c ];

    meta = {
        # ...
    };
}
