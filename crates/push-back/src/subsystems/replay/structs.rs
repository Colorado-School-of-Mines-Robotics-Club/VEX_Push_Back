use serde::{Deserialize, Serialize};
use std::{
    fs::File, io::{BufReader, BufWriter}, time::{Duration, Instant}
};

pub(super) type SerializedSubsystemStates = Vec<(String, ciborium::Value)>;

#[derive(Debug, Serialize, Deserialize)]
pub(super) struct RecordingEntry {
    pub(super) subsystem_states: SerializedSubsystemStates,
    pub(super) micros_elapsed: u32
}

#[derive(Debug)]
pub(super) enum ReplayMode {
    Replaying {
        next_entry: Option<RecordingEntry>
    },
    Recording,
}

#[derive(Debug, Default)]
pub(super) enum SubsystemState {
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
