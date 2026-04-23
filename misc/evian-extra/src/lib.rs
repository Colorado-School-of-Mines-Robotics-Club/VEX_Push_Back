#![feature(stmt_expr_attributes)]
use evian::drivetrain::model::{Differential, DrivetrainModel};

pub mod control;
pub mod math;
pub mod motion;
pub mod utils;

pub trait TankVelocity: DrivetrainModel {
	/// Drives a tank drivetrain at a specified velocity in RPMs
	fn drive_tank_velocity(&mut self, left: i32, right: i32) -> Result<(), Self::Error>;
}

impl TankVelocity for Differential {
	fn drive_tank_velocity(&mut self, left: i32, right: i32) -> Result<(), Self::Error> {
		let mut rtn = Ok(());

		let mut left_borrow = self.left.borrow_mut();
		let mut right_borrow = self.right.borrow_mut();

		for (motor, velocity) in left_borrow
			.as_mut()
			.iter_mut()
			.map(|m| (m, left))
			.chain(right_borrow.as_mut().iter_mut().map(|m| (m, right)))
		{
			let result = motor.set_velocity(velocity);

			if result.is_err() {
				rtn = result;
			}
		}

		rtn
	}
}
