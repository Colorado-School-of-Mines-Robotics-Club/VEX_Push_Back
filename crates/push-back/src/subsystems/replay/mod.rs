mod structs;

use std::{
    fs::File,
    io::{self, BufRead, BufReader, Write},
    time::{Duration, Instant},
};
use structs::*;

use vexide::controller::ControllerState;

const MAGIC: &str = "REPLAY_MAGIC!";

fn read_entry(file: &mut impl BufRead, buffer: &mut Vec<u8>) -> Option<RecordingEntry> {
    if let Err(e) = file.read_until(b'\0', buffer) {
        eprintln!("reading entry of recording should succeed: {e:?}");
        return None
    }

    match cobs::decode_in_place(buffer) {
        Ok(n) => buffer.truncate(n),
        Err(e) => {
            eprintln!("cobs decoding entry of recording should succeed: {e:?}");
            return None
        }
    }

    let parsed = match ciborium::from_reader(&buffer[..]) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("parsing entry of recording should succeed: {e:?}");
            return None
        }
    };

    Some(parsed)
}

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
            buffer: Vec::new(),
            start_time: Instant::now(),
            duration,
            previous_state: SerializedSubsystemStates::new(),
            mode: ReplayMode::Recording,
        };

        Ok(())
    }

    pub fn start_replaying(&mut self, path: &str) -> io::Result<bool> {
        let mut file = BufReader::new(File::options().read(true).open(path)?);

        // Parse magic
        let mut magic = String::new();
        let Ok(_) = file
            .read_line(&mut magic) else {
                eprintln!("Should read magic line");
                return Ok(false)
            };

        let Some((_, seconds)) = magic
            .split_once(MAGIC) else {
                eprintln!("Magic should split on '{}'", MAGIC);
                return Ok(false)
            };
        let duration = Duration::from_secs(
            seconds
                .parse()
                .expect("Magic duration parsing should succeed"),
        );

        // Read first entry
        let mut buffer = Vec::with_capacity(4096);
        let next_entry = read_entry(&mut file, &mut buffer);

        self.state = SubsystemState::Enabled {
            file: file.into_inner(),
            buffer,
            start_time: Instant::now(),
            duration,
            previous_state: SerializedSubsystemStates::new(),
            mode: ReplayMode::Replaying { next_entry },
        };

        Ok(true)
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
                buffer,
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
                        let file = BufReader::new(file);
                        let mut current_state = &*previous_state;

                        let next_timestamp = Duration::from_micros(next_entry.micros_elapsed as u64);
                        // Check if we have passed the time of the next entry, and should move to that state
                        if start_time.elapsed() > next_timestamp {
                            buffer.clear();

                            next_entry_opt = Some(next_entry)
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
