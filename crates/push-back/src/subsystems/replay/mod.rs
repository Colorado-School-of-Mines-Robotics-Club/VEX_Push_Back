pub mod structs;

use std::{
    collections::HashMap,
    fs::File,
    io::{self, BufRead, BufReader, Write},
    time::{Duration, Instant},
};
use structs::*;

use vexide::{controller::ControllerState, time::sleep};

use crate::subsystems::ControllableSubsystem;

const MAGIC: &str = "REPLAY_MAGIC!";

fn read_entry(file: &mut impl BufRead, buffer: &mut Vec<u8>) -> Option<RecordingEntry> {
    buffer.clear();
    let read_len = file.read_until(b'\0', buffer);
    if let Err(e) = read_len {
        eprintln!("reading entry of recording should succeed: {e:?}");
        return None;
    } else if let Ok(0) = read_len {
        return None;
    }

    let n = match cobs::decode_in_place(buffer) {
        Ok(n) => n,
        Err(e) => {
            eprintln!("cobs decoding entry of recording should succeed: {e:?}");
            return None;
        }
    };

    let parsed = match ciborium::from_reader(&buffer[0..n]) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("parsing entry of recording should succeed: {e:?}");
            return None;
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
        file.flush()?;

        self.state = SubsystemState::Enabled {
            file,
            buffer: Vec::new(),
            state_buffer: Vec::new(),
            start_time: Instant::now(),
            duration,
            previous_state: SerializedSubsystemStates::new(),
            mode: ReplayMode::Recording,
        };

        Ok(())
    }

    pub fn record(
        &mut self,
        controller: &ControllerState,
        states: &[(&'static str, &dyn ControllableSubsystem)],
    ) {
        let current_state = &mut self.state;
        if let SubsystemState::Enabled {
            file,
            buffer,
            state_buffer,
            start_time,
            duration,
            previous_state,
            mode,
        } = current_state
        {
            match mode {
                // Exit if the duration has passed or x pressed
                _ if start_time.elapsed() > *duration || controller.button_x.is_now_pressed() => {
                    file.flush().expect("Flushing replay file should succeed");
                    let metadata = file
                        .metadata()
                        .expect("Reading replay file metadata should succeed");
                    println!("Ended recording!\nFile size: {} bytes", metadata.len());
                    *current_state = SubsystemState::Disabled;
                }
                // If recording, write to file
                ReplayMode::Recording => {
                    state_buffer.clear();
                    state_buffer.extend(states.iter().filter_map(|(name, subsystem)| {
                        Some((name.to_string(), subsystem.state()?))
                    }));

                    if *state_buffer != *previous_state {
                        // 2^32 - 1 microseconds is well over the 2min this code needs to deal with, so discard the rest
                        let micros_elapsed = start_time.elapsed().as_micros() as u32;

                        buffer.clear();
                        ciborium::into_writer(
                            &RecordingEntry {
                                subsystem_states: state_buffer.clone(),
                                micros_elapsed,
                            },
                            &mut *buffer,
                        )
                        .expect("Serializing recording entry should succeed");
                        let unencoded_len = buffer.len();

                        let max_encoded_len = cobs::max_encoding_length(unencoded_len);
                        buffer.resize(unencoded_len + max_encoded_len, 0);

                        let (unencoded, encoded) = buffer.split_at_mut(unencoded_len);
                        let encoded_len = cobs::encode(unencoded, encoded);

                        file.write_all(&encoded[0..encoded_len])
                            .expect("Recording state should succeed");
                        file.write_all(b"\0")
                            .expect("Recording state delimeter should succeed");
                        file.flush().expect("flushing should succeed");

                        _ = std::mem::replace(previous_state, state_buffer.clone());
                    }
                }
                _ => (),
            }
        }
    }

    pub async fn replay(
        &mut self,
        file: File,
        mut subsystems: HashMap<&'static str, &mut dyn ControllableSubsystem>,
    ) {
        let mut reader = BufReader::new(file);
        let mut buffer = Vec::new();

        let mut magic = String::new();
        if let Err(e) = reader.read_line(&mut magic) {
            eprintln!("Reading magic line should succeed: {e:?}");
            return;
        };
        magic.pop(); // newline

        let Some((_, secs)) = magic.split_once(MAGIC) else {
            eprintln!("Splitting magic line should succeed: {magic}");
            return;
        };
        let Ok(secs) = secs.parse() else {
            eprintln!(
                "Parsing magic duration should succeed: {secs}, {}",
                secs.len()
            );
            return;
        };
        let duration = Duration::from_secs(secs);

        let mut next_entry = read_entry(&mut reader, &mut buffer);

        let mut current_states = None;

        let started_at = Instant::now();
        loop {
            let elapsed = started_at.elapsed();

            if elapsed > duration {
                println!("Auton finished");
                break;
            }

            if next_entry
                .as_ref()
                .is_some_and(|ne| elapsed > Duration::from_micros(ne.micros_elapsed as u64))
            {
                current_states = Some(next_entry.take().unwrap().subsystem_states);
                next_entry = read_entry(&mut reader, &mut buffer);
            }

            if let Some(ref states) = current_states {
                for (subsystem, state) in states {
                    if let Some(subsystem) = subsystems.get_mut(subsystem.as_str()) {
                        subsystem.direct(state);
                        sleep(Duration::ZERO).await // Let VexOS flush everything necessary
                    }
                }
            }

            sleep(Duration::from_millis(2)).await
        }
    }
}
