use std::time::Duration;

use evian::control::loops::Feedback;
use nalgebra::{
	Matrix2, Matrix3, Matrix3x2, OMatrix, Rotation2, SMatrix, U2, U3, Vector2, Vector3, matrix,
	vector,
};
use shrewnit::{Dimension, FeetPerSecond, Inches, Meters, MetersPerSecond, One, Scalar};

use crate::math::{brysons_rule, discretize_ab, lqr};

macro_rules! mdbg {
	($v:expr) => {{
		for row in $v.row_iter() {
			for column in row.iter() {
				print!("{}, ", column);
			}
			println!();
		}
		$v
	}};
}

#[derive(Clone)]
pub struct LTVUnicycleController {
	q: Matrix3<f64>,
	r: Matrix2<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct LTVState {
	pub position: Vector2<f64>,
	pub heading: f64,
	pub velocity: f64,
}

impl From<LTVState> for Vector3<f64> {
	fn from(value: LTVState) -> Self {
		vector![value.position.x, value.position.y, value.heading]
	}
}

impl LTVUnicycleController {
	// Units in inches and radians
	pub const FRC_DEFAULT_Q: Matrix3<f64> = matrix![
		0.16516096, 0.0,        0.0;  // 0.0625 m
		0.0,        0.04129024, 0.0;  // 0.125  m
		0.0,        0.0,        0.25; // 2.0 rad
	];
	pub const FRC_DEFAULT_R: Matrix2<f64> = matrix![
		0.00064516, 0.0;  // 1 m/s
		0.0,        0.25; // 2 rad/s
	];

	pub const fn new(q: &Matrix3<f64>, r: &Matrix2<f64>) -> Self {
		Self { q: *q, r: *r }
	}

	pub fn new_brysons_rule(
		acceptable_state_error: &Vector3<f64>,
		acceptable_control_effort: &Vector2<f64>,
	) -> Self {
		Self::new(
			&brysons_rule(acceptable_state_error),
			&brysons_rule(acceptable_control_effort),
		)
	}

	pub const fn new_with_frc_defaults() -> Self {
		Self::new(&Self::FRC_DEFAULT_Q, &Self::FRC_DEFAULT_R)
	}
}

impl Feedback for LTVUnicycleController {
	type State = LTVState;
	type Signal = Vector2<f64>;

	fn update(
		&mut self,
		measurement: Self::State,
		setpoint: Self::State,
		dt: Duration,
	) -> Self::Signal {
		let (a, b) = discretize_ab(
			&matrix![
				0.0, 0.0, 0.0;
				0.0, 0.0, setpoint.velocity.max(1e-4);
				0.0, 0.0, 0.0;
			],
			&matrix![
				1.0, 0.0;
				0.0, 0.0;
				0.0, 1.0;
			],
			dt.as_secs_f64(),
		);
		let k = lqr(&a, &b, &self.q, &self.r, &Matrix3x2::<f64>::zeros(), 1e-10);

		let rotation = Rotation2::new(-measurement.heading);
		let m = SMatrix::<_, 3, 3>::from_fn(|i, j| {
			if i < 2 && j < 2 {
				rotation[(i, j)]
			} else if i == j {
				1.0
			} else {
				0.0
			}
		});

		let setpoint: Vector3<f64> = setpoint.into();
		let measurement: Vector3<f64> = measurement.into();

		k * m * (setpoint - measurement)
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn test_controller() {
		let mut controller = LTVUnicycleController::new_with_frc_defaults();

		dbg!(controller.update(
			LTVState {
				// State
				position: vector![0.0, 0.0],
				heading: f64::to_radians(90.0),
				velocity: 0.0,
			},
			LTVState {
				// Setpoint
				position: vector![0.0, 1.0],
				heading: f64::to_radians(90.0),
				velocity: 0.0,
			},
			Duration::from_millis(5),
		));
	}
}
