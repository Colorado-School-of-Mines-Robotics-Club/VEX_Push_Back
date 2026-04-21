use shrewnit::{
	Angle, AngularAcceleration, AngularVelocity, Degrees, DegreesPerSecond,
	DegreesPerSecondSquared, Length, LinearAcceleration, LinearVelocity, Meters, MetersPerSecond,
	MetersPerSecondSquared,
};

// This has used the flawed value for ages so fixing it now would break all our autons
// Next year this'll be good trust
const _FLAWED_OTOS_I16_MAX: f64 = i16::MAX as f64;
const _CORRECT_OTOS_I16_MAX: f64 = i16::MAX as f64 + 1.0;
const OTOS_I16_MAX: f64 = _FLAWED_OTOS_I16_MAX;

// Shrewnit simple_unit!() doesn't support X per Y canonical so this is a workaround
macro_rules! impl_otos_reg_unit {
    ($($name:ident ($dimension:ident): $max:literal $base:ident),+) => {
        $(
            impl_otos_reg_unit!($name, $dimension, $max, $base);
        )+
    };
    ($name:ident, $dimension:ident, $max:literal, $base:ident) => {
        shrewnit::unit_type!(pub $name of dimension $dimension);

        impl<S: shrewnit::Scalar> shrewnit::UnitOf<S, $dimension<S>> for $name {
            fn to_canonical(converted: S) -> S {
                converted / S::from_f64(OTOS_I16_MAX).unwrap()
                    * $base::to_canonical(S::from_f64($max as f64).unwrap())
            }

            fn from_canonical(canonical: S) -> S {
                canonical / $base::to_canonical(S::from_f64($max as f64).unwrap())
                    * S::from_f64(OTOS_I16_MAX).unwrap()
            }
        }

        impl shrewnit::One<f64, $dimension<f64>> for $name {
            const ONE: $dimension<f64> = $dimension::from_canonical(
                { $base::ONE as $dimension }
                    .mul_scalar($max as f64)
                    .canonical()
                    / (OTOS_I16_MAX),
            );
            // TODO: address rounding problems
            const ONE_CANONICAL: f64 = $dimension::<f64>::from_canonical(1.0)
                .to::<$base>()
                / ($max as f64)
                * (OTOS_I16_MAX);
        }
    };
}

// Implement all of the OTOS register units
impl_otos_reg_unit!(
	OtosLength (Length):                           10      Meters,
	OtosAngle (Angle):                             180     Degrees,
	OtosLinearVelocity (LinearVelocity):           5       MetersPerSecond,
	OtosAngularVelocity (AngularVelocity):         2_000   DegreesPerSecond,
	OtosLinearAcceleration (LinearAcceleration):   157     MetersPerSecondSquared,
	OtosAngularAcceleration (AngularAcceleration): 180_000 DegreesPerSecondSquared
);

#[cfg(test)]
mod tests {
	use shrewnit::{Inches, One};

	use super::*;

	// Values taken from https://github.com/sparkfun/qwiic_otos_py/blob/dcd21356f0e130a23c3444ba924900aa494c31a2/qwiic_otos.py#L158-L168
	const METER_TO_I16: f64 = 32768.0 / 10.0;
	const MPS_TO_I16: f64 = 32768.0 / 5.0;

	const LENGTHS: &[f64] = &[1.0, 2.25, 6.0, 10.0];
	const VELOCITIES: &[f64] = &[1.0, 2.78, 3.0, 4.783];

	// THESE FAIL, CURRENT CODE IS INCORRECT BUT FIXING IT WOULD BREAK ALL OUR AUTONS SO LOL
	#[test]
	fn test_conversions() {
		for len in LENGTHS {
			assert_eq!((*len * Meters).to::<OtosLength>(), len * METER_TO_I16);
		}
		for vel in VELOCITIES {
			assert_eq!(
				(*vel * MetersPerSecond).to::<OtosLinearVelocity>(),
				vel * MPS_TO_I16
			);
		}
	}

	#[test]
	fn test_one() {
		assert_eq!(OtosLength::ONE.to::<Meters>(), 1.0 / METER_TO_I16);
		assert_eq!(OtosLength::ONE_CANONICAL, 1.0 * METER_TO_I16);
	}
}
