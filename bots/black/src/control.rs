pub mod basic {
    use core::time::Duration;

    use evian::{
        control::loops::{AngularPid, Pid}, math::Angle, motion::{Basic, Seeking}, prelude::Tolerances
    };

    pub const CONTROLLER: Basic<Pid, AngularPid> = Basic {
        linear_controller: LINEAR_PID,
        angular_controller: ANGULAR_PID,
        linear_tolerances: LINEAR_TOLERANCES,
        angular_tolerances: ANGULAR_TOLERANCES,
        timeout: Some(Duration::from_secs(5)),
    };

    pub const LINEAR_PID: Pid = Pid::new(0.025, 0.001, 0.0005, None);
    pub const ANGULAR_PID: AngularPid = AngularPid::new(0.65, 0.01, 0.04, None);

    // TODO: actually check these
    pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(1.0)
        .velocity(0.5)
        .duration(Duration::from_millis(15));
    pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
        .error(Angle::from_degrees(5.0).as_radians())
        .velocity(Angle::from_degrees(0.15).as_radians())
        .duration(Duration::from_millis(15));
}
