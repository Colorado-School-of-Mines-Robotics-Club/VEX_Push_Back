use std::{
	fs::{File, OpenOptions},
	io::{self, Write},
	path::Path,
	time::Instant,
};

use evian::{
	prelude::{TracksHeading, TracksPosition, TracksVelocity},
	tracking::Tracking,
};

use crate::log::TelemetryLog;

pub mod log;

pub struct TelemetryLogger {
	file: File,
	start: Instant,
}

impl TelemetryLogger {
	/// Makes a new logger using a directory. Will make log files in the format DIR/{N}, starting with 0
	pub fn new(dir: &Path) -> io::Result<Self> {
		let idx_file = dir.join("idx");
		let last_index = std::fs::read_to_string(&idx_file)
			.ok()
			.and_then(|s| s.parse::<u32>().ok());
		let current_index = last_index.map(|n| n + 1).unwrap_or(0).to_string();
		_ = std::fs::write(idx_file, &current_index);

		OpenOptions::new()
			.create(true)
			.truncate(true)
			.write(true)
			.open(dir.join(current_index))
			.map(Self::new_with_file)
	}

	pub fn new_with_file(file: File) -> Self {
		Self {
			file,
			start: Instant::now(),
		}
	}

	pub fn log<T>(&mut self, tracking: T)
	where
		T: Tracking + TracksPosition + TracksVelocity + TracksHeading,
	{
		let encoded = bitcode::encode(&TelemetryLog::from_tracking(self.start.elapsed(), tracking));
		let cobs_encoded = cobs::encode_vec(&encoded);

		_ = self.file.write_all(&cobs_encoded);
		_ = self.file.write_all(b"\0");
	}
}
