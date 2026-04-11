use std::time::Duration;

use evian::control::loops::Feedback;
use nalgebra::{
	Matrix2, Matrix3, OMatrix, Rotation2, SMatrix, U2, U3, Vector2, Vector3, matrix, vector,
};

use crate::math::{brysons_rule, discretize_ab, lqr};

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
	// In inches because thats what we use, todo: units
	pub const FRC_DEFAULT_Q: Vector3<f64> = vector![
		2.460629921, // 0.0625 m
		4.921259843, // 0.125  m
		2.000000000, // 2 rad
	];
	pub const FRC_DEFAULT_R: Vector2<f64> = vector![
		39.37007874, // 1 m/s
		2.000000000, // 2 rad/s
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
		let k = lqr(
			&a,
			&b,
			&self.q,
			&self.r,
			&OMatrix::<f64, U3, U2>::zeros(),
			1e-10,
		);

		let rotation = Rotation2::new(measurement.heading);
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
