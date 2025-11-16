mod structs;

use bytemuck::Zeroable as _;
use std::{
    fs::File,
    io::{self, BufRead, BufReader, ErrorKind, Read, Write},
    time::{Duration, Instant},
};
use structs::*;

use vexide::controller::ControllerState;

const MAGIC: &str = "REPLAY_MAGIC!";

#[derive(Default)]
pub struct ReplaySubsystem {
    state: SubsystemState,
}

impl ReplaySubsystem {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start_recording(&mut self, path: &str, duration: Duration) -> io::Result<()> {
        let mut file = File::options()
            .create(true)
            .write(true)
            .truncate(true)
            .open(path)?;

        file.write_all(MAGIC.as_bytes())?;
        file.write_all(format!("{}", duration.as_secs()).as_bytes())?;
        file.write_all(b"\n")?;

        self.state = SubsystemState::Enabled {
            file,
            start_time: Instant::now(),
            duration,
            previous_state: SerializedControllerState::zeroed(),
            mode: ReplayMode::Recording,
        };

        Ok(())
    }

    pub fn start_replaying(&mut self, path: &str) -> io::Result<()> {
        let mut file = File::options().read(true).open(path)?;

        // Parse magic
        let mut magic = String::new();
        BufReader::new(&mut file)
            .read_line(&mut magic)
            .expect("Reading magic line should succeed");
        let (_, seconds) = magic
            .split_once(MAGIC)
            .unwrap_or_else(|| panic!("Magic should split on '{}'", MAGIC));
        let duration = Duration::from_secs(
            seconds
                .parse()
                .expect("Magic duration parsing should succeed"),
        );

        // Read first entry
        let mut entry = [0u8; size_of::<RecordingEntry>()];
        let next_entry = match file.read_exact(&mut entry) {
            Ok(_) => Some(bytemuck::must_cast(entry)),
            Err(e) if e.kind() == ErrorKind::UnexpectedEof => None,
            Err(e) => panic!("reading first entry of recording should succeed: {e:?}"),
        };

        self.state = SubsystemState::Enabled {
            file,
            start_time: Instant::now(),
            duration,
            previous_state: SerializedControllerState::zeroed(),
            mode: ReplayMode::Replaying { next_entry },
        };

        Ok(())
    }

    pub fn control(&mut self, controller: &mut ControllerState) {
        let current_state = &mut self.state;
        match current_state {
            SubsystemState::Disabled => {
                // If disabled, watch for left (30s record) or right (2min record)
                if controller.button_left.is_now_pressed() {
                    println!("Starting 30s recording!");
                    self.start_recording("record30.txt", Duration::from_secs(30))
                        .expect("Starting relay should succeed");
                } else if controller.button_right.is_now_pressed() {
                    println!("Starting 120s recording!");
                    self.start_recording("record120.txt", Duration::from_secs(120))
                        .expect("Starting relay should succeed");
                }
            }
            SubsystemState::Enabled {
                file,
                start_time,
                duration,
                previous_state,
                mode,
            } => match mode {
                // Exit if the duration has passed or x pressed
                _ if start_time.elapsed() > *duration || controller.button_x.is_now_pressed() => {
                    file.flush().expect("Flushing replay file should succeed");
                    let metadata = file
                        .metadata()
                        .expect("Reading replay file metadata should succeed");
                    println!("Ended recording!\nFile size: {} bytes", metadata.len());
                    *current_state = SubsystemState::Disabled;
                }
                // If replaying, control the bot
                ReplayMode::Replaying {
                    next_entry: next_entry_opt,
                } => match next_entry_opt {
                    Some(next_entry) => {
                        let next_entry = *next_entry; // copy to avoid mutable reference issues
                        let mut current_state = &*previous_state;

                        let next_timestamp = bytemuck::must_cast::<_, u64>([
                            next_entry.micros_elapsed_low,
                            next_entry.micros_elapsed_high,
                            0,
                            0,
                        ]);
                        // Check if we have passed the time of the next entry, and should move to that state
                        if start_time.elapsed() > Duration::from_micros(next_timestamp) {
                            let mut buf = [0u8; size_of::<RecordingEntry>()];
                            match file.read_exact(&mut buf) {
                                Ok(_) => {
                                    current_state = &next_entry.controller_state;
                                    *next_entry_opt = Some(bytemuck::must_cast(buf));
                                }
                                Err(e) if e.kind() == ErrorKind::UnexpectedEof => {
                                    *next_entry_opt = None
                                }
                                Err(e) => {
                                    panic!("reading next entry of recording should succeed: {e:?}")
                                }
                            }
                        }

                        *controller = current_state.as_controller_state(previous_state);

                        *previous_state = *current_state;
                    }
                    None => *controller = previous_state.as_controller_state(previous_state),
                },
                // If recording, write to file
                ReplayMode::Recording => {
                    let state: SerializedControllerState = (&*controller).into();

                    if state != *previous_state {
                        // 2^32 - 1 microseconds is well over the 2min this code needs to deal with, so discard the rest
                        let [low, high] = bytemuck::must_cast_slice::<_, u16>(&[start_time
                            .elapsed()
                            .as_micros()])[0..2]
                        else {
                            const {
                                // Ensure first two u16s are the correct ones
                                assert!(cfg!(target_endian = "little"));
                                // Ensure casting the microseconds result results in at least two u16s
                                assert!(
                                    size_of_val(&Duration::new(0, 0).as_micros())
                                        >= 2 * size_of::<u16>()
                                );
                            }
                            unreachable!()
                        };

                        file.write_all(bytemuck::must_cast_slice(&[RecordingEntry {
                            micros_elapsed_high: high,
                            micros_elapsed_low: low,
                            controller_state: state,
                        }]))
                        .expect("Recording state should succeed");
                        file.write_all(b"\n")
                            .expect("Recording state delimeter should succeed");

                        *previous_state = state;
                    }
                }
            },
        }
    }
}
