/// The common trait that should be implemented by all subsystems
pub trait Subsystem {
	/// The periodic function is run
	fn periodic();

	fn state<'a>() -> Option<&'a [u8]> {
		None
	}
}
