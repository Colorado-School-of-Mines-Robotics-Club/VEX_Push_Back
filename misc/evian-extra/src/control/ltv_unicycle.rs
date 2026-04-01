use std::time::Duration;

use evian::control::loops::Feedback;
use nalgebra::{
	Matrix2, Matrix3, OMatrix, Rotation2, U2, U3, Vector2, Vector3, matrix, stack, vector,
};

use crate::math::{brysons_rule, discretize_ab, lqr};

#[derive(Clone)]
pub struct LTVUnicycleController {
	q: Matrix3<f64>,
	r: Matrix2<f64>,
}

impl LTVUnicycleController {
	// In inches because thats what we use, todo: units
	pub const FRC_DEFAULT_Q: Vector3<f64> = vector![
		2.46, // 0.0625 m
		4.92, // 0.125  m
		2.00, // 2 rad
	];
	pub const FRC_DEFAULT_R: Vector2<f64> = vector![
		39.37, // 1 m/s
		2.00,  // 2 rad/s
	];
	pub fn new(q: &Vector3<f64>, r: &Vector2<f64>) -> Self {
		Self {
			q: brysons_rule(q),
			r: brysons_rule(r),
		}
	}

	pub fn new_with_frc_defaults() -> Self {
		Self::new(&Self::FRC_DEFAULT_Q, &Self::FRC_DEFAULT_R)
	}
}

impl LTVUnicycleController {
	fn calculate(
		&mut self,
		measurement: Self::State,
		setpoint: Self::State,
		dt: Duration,
	) -> Self::Signal {
		let v = setpoint.z;
		let (a, b) = discretize_ab(
			&matrix![
				0.0, 0.0, 0.0;
				0.0, 0.0, v;
				0.0, 0.0, 0.0;
			],
			&matrix![
				1.0, 0.0;
				0.0, 0.0;
				0.0, 1.0;
			],
			dt.as_secs_f64(),
		);
		let k = lqr(
			&a,
			&b,
			&self.q,
			&self.r,
			&OMatrix::<f64, U3, U2>::zeros(),
			1e-10,
		);

		let rotation = Rotation2::new(measurement.z);
		#[rustfmt::skip]
		let m = stack![
			rotation.matrix(),            0;
			0,                 matrix![1.0];
		];

		k * m * (setpoint - measurement)
	}
}
