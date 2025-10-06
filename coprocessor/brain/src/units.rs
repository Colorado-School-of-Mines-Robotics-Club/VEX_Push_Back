use shrewnit::{
    Angle, AngularAcceleration, AngularVelocity, Degrees, DegreesPerSecond,
    DegreesPerSecondSquared, Length, LinearAcceleration, LinearVelocity, Meters, MetersPerSecond,
    MetersPerSecondSquared,
};

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
                converted / S::from_i16(i16::MAX).unwrap()
                    * $base::to_canonical(S::from_f64($max as f64).unwrap())
            }

            fn from_canonical(canonical: S) -> S {
                canonical / $base::to_canonical(S::from_f64($max as f64).unwrap())
                    * S::from_i16(i16::MAX).unwrap()
            }
        }

        impl shrewnit::One<f64, $dimension<f64>> for $name {
            const ONE: $dimension<f64> = $dimension::from_canonical(
                { $base::ONE as $dimension }
                    .mul_scalar($max as f64)
                    .canonical()
                    / (i16::MAX as f64),
            );
            const ONE_CANONICAL: f64 = { $base::ONE as $dimension }
                .mul_scalar($max as f64)
                .canonical()
                / (i16::MAX as f64);
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
