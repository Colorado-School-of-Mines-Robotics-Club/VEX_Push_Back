use bitflags::bitflags;
use bytemuck::{Pod, Zeroable};
use std::{
    fs::File,
    time::{Duration, Instant},
};
use vexide::controller::{ButtonState, ControllerState, JoystickState};

// This is very safe source: trust me bro
struct ButtonStateWrapper {
    _prev_is_pressed: bool,
    _is_pressed: bool,
}

impl From<ButtonStateWrapper> for ButtonState {
    fn from(value: ButtonStateWrapper) -> Self {
        const {
            assert!(size_of::<ButtonStateWrapper>() == size_of::<ButtonState>());
            assert!(align_of::<ButtonStateWrapper>() == align_of::<ButtonState>());
        }
        unsafe { std::mem::transmute(value) }
    }
}

struct JoystickStateWrapper {
    _x_raw: i8,
    _y_raw: i8,
}

impl From<JoystickStateWrapper> for JoystickState {
    fn from(value: JoystickStateWrapper) -> Self {
        const {
            assert!(size_of::<JoystickStateWrapper>() == size_of::<JoystickState>());
            assert!(align_of::<JoystickStateWrapper>() == align_of::<JoystickState>());
        }
        unsafe { std::mem::transmute(value) }
    }
}

bitflags! {
    #[derive(Clone, Copy, PartialEq, Eq, Debug, Pod, Zeroable)]
    #[repr(transparent)]
    pub(super) struct ControllerStateFlags: u16 {
        const A     = 0b0000_0000_0000_0001;
        const B     = 0b0000_0000_0000_0010;
        const X     = 0b0000_0000_0000_0100;
        const Y     = 0b0000_0000_0000_1000;

        const UP    = 0b0000_0000_0001_0000;
        const DOWN  = 0b0000_0000_0010_0000;
        const LEFT  = 0b0000_0000_0100_0000;
        const RIGHT = 0b0000_0000_1000_0000;

        const L1    = 0b0000_0001_0000_0000;
        const L2    = 0b0000_0010_0000_0000;
        const R1    = 0b0000_0100_0000_0000;
        const R2    = 0b0000_1000_0000_0000;

        const POWER = 0b0001_0000_0000_0000;
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Zeroable, Pod)]
#[repr(C)]
pub(super) struct SerializedControllerState {
    pub(super) left_stick_x: i8,
    pub(super) left_stick_y: i8,
    pub(super) right_stick_x: i8,
    pub(super) right_stick_y: i8,
    pub(super) button_flags: ControllerStateFlags,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Zeroable, Pod)]
#[repr(C)]
pub(super) struct RecordingEntry {
    pub(super) controller_state: SerializedControllerState,
    // Padding :(
    pub(super) micros_elapsed_high: u16,
    pub(super) micros_elapsed_low: u16,
}

impl From<&ControllerState> for SerializedControllerState {
    fn from(state: &ControllerState) -> Self {
        let mut button_flags = ControllerStateFlags::empty();

        button_flags.set(ControllerStateFlags::A, state.button_a.is_pressed());
        button_flags.set(ControllerStateFlags::B, state.button_b.is_pressed());
        button_flags.set(ControllerStateFlags::X, state.button_x.is_pressed());
        button_flags.set(ControllerStateFlags::Y, state.button_y.is_pressed());

        button_flags.set(ControllerStateFlags::UP, state.button_up.is_pressed());
        button_flags.set(ControllerStateFlags::DOWN, state.button_down.is_pressed());
        button_flags.set(ControllerStateFlags::LEFT, state.button_left.is_pressed());
        button_flags.set(ControllerStateFlags::RIGHT, state.button_right.is_pressed());

        button_flags.set(ControllerStateFlags::L1, state.button_l1.is_pressed());
        button_flags.set(ControllerStateFlags::L2, state.button_l2.is_pressed());
        button_flags.set(ControllerStateFlags::R1, state.button_r1.is_pressed());
        button_flags.set(ControllerStateFlags::R2, state.button_r2.is_pressed());

        button_flags.set(ControllerStateFlags::POWER, state.button_power.is_pressed());

        SerializedControllerState {
            left_stick_x: state.left_stick.x_raw(),
            left_stick_y: state.left_stick.y_raw(),
            right_stick_x: state.right_stick.x_raw(),
            right_stick_y: state.right_stick.y_raw(),
            button_flags,
        }
    }
}

impl SerializedControllerState {
    pub(super) fn as_controller_state(
        &self,
        previous: &SerializedControllerState,
    ) -> ControllerState {
        ControllerState {
            left_stick: JoystickStateWrapper {
                _x_raw: previous.left_stick_x,
                _y_raw: previous.left_stick_y,
            }
            .into(),
            right_stick: JoystickStateWrapper {
                _x_raw: previous.right_stick_x,
                _y_raw: previous.right_stick_y,
            }
            .into(),
            button_a: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::A.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::A.intersects(previous.button_flags),
            }
            .into(),
            button_b: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::B.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::B.intersects(previous.button_flags),
            }
            .into(),
            button_x: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::X.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::X.intersects(previous.button_flags),
            }
            .into(),
            button_y: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::Y.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::Y.intersects(previous.button_flags),
            }
            .into(),
            button_up: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::UP.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::UP.intersects(previous.button_flags),
            }
            .into(),
            button_down: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::DOWN.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::DOWN.intersects(previous.button_flags),
            }
            .into(),
            button_left: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::LEFT.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::LEFT.intersects(previous.button_flags),
            }
            .into(),
            button_right: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::RIGHT.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::RIGHT.intersects(previous.button_flags),
            }
            .into(),
            button_l1: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::L1.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::L1.intersects(previous.button_flags),
            }
            .into(),
            button_l2: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::L2.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::L2.intersects(previous.button_flags),
            }
            .into(),
            button_r1: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::R1.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::R1.intersects(previous.button_flags),
            }
            .into(),
            button_r2: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::R2.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::R2.intersects(previous.button_flags),
            }
            .into(),
            button_power: ButtonStateWrapper {
                _is_pressed: ControllerStateFlags::POWER.intersects(self.button_flags),
                _prev_is_pressed: ControllerStateFlags::POWER.intersects(previous.button_flags),
            }
            .into(),
        }
    }
}

#[derive(Debug)]
pub(super) enum ReplayMode {
    Replaying { next_entry: Option<RecordingEntry> },
    Recording,
}

#[derive(Debug, Default)]
pub(super) enum SubsystemState {
    #[default]
    Disabled,
    Enabled {
        /// The file storing the replay data
        file: File,
        /// When the replay subsystem was enabled
        start_time: Instant,
        /// How long this replay session runs for
        duration: Duration,
        /// The previous state of the controller
        previous_state: SerializedControllerState,
        /// Whether the subsystem is recording or replaying
        mode: ReplayMode,
    },
}
