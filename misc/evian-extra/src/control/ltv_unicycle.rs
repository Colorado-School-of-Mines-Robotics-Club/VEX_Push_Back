use std::time::Duration;

use evian::control::loops::Feedback;
use nalgebra::{Matrix2, Matrix3, Matrix3x2, Rotation2, SMatrix, Vector2, Vector3, matrix};

use crate::math::{brysons_rule, discretize_ab, lqr};

#[derive(Clone)]
pub struct LTVUnicycleController {
	q: Matrix3<f64>,
	r: Matrix2<f64>,
}

#[derive(Debug, Clone, Copy)]
pub struct LTVState {
	pub position: Vector3<f64>,
	pub velocity: Vector2<f64>,
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
		// Create the differential drivetrain model linearized around the desired velocity, and discretize it
		let (a, b) = discretize_ab(
			&matrix![
				0.0, 0.0, 0.0;
				0.0, 0.0, setpoint.velocity.x.max(1e-4);
				0.0, 0.0, 0.0;
			],
			&matrix![
				1.0, 0.0;
				0.0, 0.0;
				0.0, 1.0;
			],
			dt.as_secs_f64(),
		);

		// Calculate the LQR gain given the model and cost functions
		let k = lqr(&a, &b, &self.q, &self.r, &Matrix3x2::<f64>::zeros(), 1e-10);

		// Create the rotation matrix to transform the measurements to the correct reference
		let rotation = Rotation2::new(-measurement.position.z);
		let m = SMatrix::<_, 3, 3>::from_fn(|i, j| {
			if i < 2 && j < 2 {
				rotation[(i, j)]
			} else if i == j {
				1.0
			} else {
				0.0
			}
		});

		// Use LQR gain to calculate the ideal control input (change in velocity) for the error
		let u = k * m * (setpoint.position - measurement.position);

		// Return the desired velocity + feedback control input
		setpoint.velocity + u
	}
}
