use evian::{
	control::loops::Feedback,
	drivetrain::model::DrivetrainModel,
	math::Angle,
	motion::Basic,
	prelude::{Drivetrain, Tank, TracksForwardTravel, TracksHeading, TracksVelocity},
	tracking::Tracking,
};

pub async fn test_linear<L, A, M, T>(
	drivetrain: &mut Drivetrain<M, T>,
	basic: &mut Basic<L, A>,
	distance: f64,
) where
	L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
	A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
	M: DrivetrainModel + Tank,
	T: Tracking + TracksForwardTravel + TracksVelocity + TracksHeading,
{
	let start = drivetrain.tracking.forward_travel();

	basic.drive_distance(drivetrain, distance).await;

	let end = drivetrain.tracking.forward_travel();

	println!(
		"---------------------------\nStart: {start:.03}\nEnd:   {end:.03}\nDiff:  {diff}\nError: {error}",
		start = start,
		end = end,
		diff = (end - start),
		error = (end - start) - distance
	)
}

pub async fn test_angular<L, A, M, T>(
	drivetrain: &mut Drivetrain<M, T>,
	basic: &mut Basic<L, A>,
	distance: Angle,
) where
	L: Feedback<State = f64, Signal = f64> + Unpin + Clone,
	A: Feedback<State = Angle, Signal = f64> + Unpin + Clone,
	M: DrivetrainModel + Tank,
	T: Tracking + TracksForwardTravel + TracksVelocity + TracksHeading,
{
	let start = drivetrain.tracking.heading();
	let target = (start + distance).wrapped_full();

	basic.turn_to_heading(drivetrain, target).await;

	let end = drivetrain.tracking.heading();

	println!(
		"---------------------------\nStart: {start:.03}deg\nEnd:   {end:.03}deg\nDiff:  {diff}deg\nError: {error}deg",
		start = start.as_degrees(),
		end = end.as_degrees(),
		diff = -(end - start).wrapped_half().as_degrees(),
		error = -((end - start) - distance).wrapped_half().as_degrees()
	)
}
