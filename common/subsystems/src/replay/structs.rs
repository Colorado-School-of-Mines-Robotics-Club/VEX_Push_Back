use serde::{Deserialize, Serialize};
use std::{
	fs::File,
	time::{Duration, Instant},
};

pub type SerializedSubsystemStates = Vec<(String, ciborium::Value)>;

#[derive(Debug, Serialize, Deserialize)]
pub struct RecordingEntry {
	pub subsystem_states: SerializedSubsystemStates,
	pub micros_elapsed: u32,
}

#[derive(Debug)]
pub enum ReplayMode {
	Replaying { next_entry: Option<RecordingEntry> },
	Recording,
}

#[derive(Debug, Default)]
pub enum SubsystemState {
	#[default]
	Disabled,
	Enabled {
		/// The file storing the replay data
		file: File,
		/// A buffer to be used for reading from files
		buffer: Vec<u8>,
		/// A buffer to be used for storing subsystem states
		state_buffer: Vec<(String, ciborium::Value)>,
		/// When the replay subsystem was enabled
		start_time: Instant,
		/// How long this replay session runs for
		duration: Duration,
		/// The previous state of the controller
		previous_state: SerializedSubsystemStates,
		/// Whether the subsystem is recording or replaying
		mode: ReplayMode,
	},
}
