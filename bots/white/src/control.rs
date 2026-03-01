use core::time::Duration;

use evian::{
	control::loops::{AngularPid, Pid},
	math::Angle,
	motion::{Basic, Seeking},
	prelude::Tolerances,
};

pub const BASIC_CONTROLLER: Basic<Pid, AngularPid> = Basic {
	linear_controller: LINEAR_PID,
	angular_controller: ANGULAR_PID,
	linear_tolerances: LINEAR_TOLERANCES,
	angular_tolerances: ANGULAR_TOLERANCES,
	timeout: Some(Duration::from_secs(5)),
};
pub const SEEKING_CONTROLLER: Seeking<Pid, Pid> = Seeking {
	linear_controller: LINEAR_PID,
	lateral_controller: LATERAL_PID,
	tolerances: LINEAR_TOLERANCES,
	timeout: Some(Duration::from_secs(5)),
};

pub const LINEAR_PID: Pid = {
	let mut pid = Pid::new(0.031, 0.00, 0.00050, Some(2.0));
	pid.set_output_limit(Some(0.60));
	pid
};
pub const ANGULAR_PID: AngularPid = {
	let mut pid = AngularPid::new(0.41, 0.005, 0.02, Some(Angle::from_degrees(2.0)));
	// pid.set_output_limit(Some(0.5));
	pid
};
// pub const LATERAL_PID: Pid = {
// 	let mut pid = Pid::new(0.023, 0.015, 0.00090, Some(1.0));
// 	// pid.set_output_limit(Some(0.60));
// 	pid
// };
pub const LATERAL_PID: Pid = {
	let mut pid = Pid::new(0.00927, 0.00, 0.0001, Some(1.0));
	// pid.set_output_limit(Some(0.60));
	pid
};

pub const LINEAR_TOLERANCES: Tolerances = Tolerances::new()
	.error(0.50)
	// .velocity(0.50)
	.duration(Duration::from_millis(15));
pub const ANGULAR_TOLERANCES: Tolerances = Tolerances::new()
	.error(Angle::from_degrees(1.5).as_radians())
	.duration(Duration::from_millis(15));
