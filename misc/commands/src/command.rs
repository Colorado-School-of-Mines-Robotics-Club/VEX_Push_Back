#[async_trait::async_trait]
pub trait Command {
	/// Runs once when a [Command] is started.
	///
	/// This should be used for actions that need to run only once after this command activates, such as triggering pneumatics
	async fn initialize(&mut self) {}

	/// Run repeatedly until this command is either finished or canceled
	///
	/// This should be used for actions that need done repeatedly throughout the command lifetime, such as setting motor outputs to joystick inputs
	async fn execute(&mut self) {}

	/// Runs once when a [Command] is finsihed or canceled.
	///
	/// This should be used to clean-up state, such as setting drivetrain motors to zero
	async fn end(&mut self) {}

	/// Should return whether or not this command is finished.
	///
	/// As soon as this returns true, [Command::end] is executed and the [Command] is dropped
	fn finished(&self) -> bool {
		false
	}
}
