pub mod basic {
	use core::time::Duration;

	use evian::{
		control::loops::{AngularPid, Pid},
		math::Angle,
		motion::Basic,
		prelude::Tolerances,
	};

	pub const CONTROLLER: Basic<Pid, AngularPid> = Basic {
		linear_controller: LINEAR_PID,
		angular_controller: ANGULAR_PID,
		linear_tolerances: LINEAR_TOLERANCES,
		angular_tolerances: ANGULAR_TOLERANCES,
		timeout: Some(Duration::from_secs(5)),
	};

	pub const LINEAR_PID: Pid = {
		let mut pid = Pid::new(0.031, 0.00, 0.00050, Some(2.0));
		pid.set_output_limit(Some(0.60));
		pid
	};
	pub const ANGULAR_PID: AngularPid = {
		let mut pid = AngularPid::new(0.25, 0.00, 0.00, Some(Angle::from_degrees(15.0)));
		pid.set_output_limit(Some(0.5));
		pid
	};

	// pub const LINEAR_PID: Pid = Pid::new(0.032, 0.0025, 0.0005, None);
	// pub const ANGULAR_PID: AngularPid = AngularPid::new(0.65, 0.01, 0.04, None);

	pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
		.error(0.50)
		// .velocity(0.50)
		.duration(Duration::from_millis(15));
	pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
		.error(Angle::from_degrees(1.5).as_radians())
		.velocity(Angle::from_degrees(1.0).as_radians())
		.duration(Duration::from_millis(15));
}
