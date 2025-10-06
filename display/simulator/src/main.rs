use display::{
    AutonRoute, AutonSelector, RobotDisplay, RobotDisplayTouchState, RobotDisplayTouchStatus,
};
use embedded_graphics_core::{
    pixelcolor::Rgb888,
    prelude::{RgbColor, Size, WebColors},
};
use embedded_graphics_simulator::{
    OutputSettingsBuilder, SimulatorDisplay, SimulatorEvent, Window,
    sdl2::{Keycode, MouseButton},
};

fn main() {
    let target = SimulatorDisplay::<Rgb888>::new(Size::new(480, 240));
    let mut display = RobotDisplay::new(target);

    let autons = [
        AutonRoute {
            text: "Red rush",
            color: Rgb888::RED,
        },
        AutonRoute {
            text: "Red chill",
            color: Rgb888::CSS_DARK_RED,
        },
        AutonRoute {
            text: "Blue rush",
            color: Rgb888::BLUE,
        },
        AutonRoute {
            text: "Blue chill",
            color: Rgb888::CSS_DARK_BLUE,
        },
        AutonRoute {
            text: "This is a very long one, does it fit? I think it does! That is so very cool",
            color: Rgb888::CSS_ORANGE,
        },
    ];

    let output_settings = OutputSettingsBuilder::new().scale(3).build();

    let mut window = Window::new("Hello World", &output_settings);
    window.set_max_fps(60);
    window.update(display.target());

    let mut auton_selector = AutonSelector::<3, 2, _, ()>::new(autons);

    'outer: loop {
        // Handle events
        // TODO: this code is horrible please don't look at it
        let mut touch_state = None;
        let prev_touch_state = display.touch_status();
        for event in window.events() {
            touch_state = match event {
                SimulatorEvent::MouseButtonUp {
                    mouse_btn: MouseButton::Left,
                    point,
                } => Some(RobotDisplayTouchStatus {
                    point,
                    state: RobotDisplayTouchState::Released,
                }),
                SimulatorEvent::MouseButtonDown {
                    mouse_btn: MouseButton::Left,
                    point,
                } => Some(RobotDisplayTouchStatus {
                    point,
                    state: RobotDisplayTouchState::Pressed,
                }),
                SimulatorEvent::MouseMove { point }
                    if matches!(
                        prev_touch_state.state,
                        RobotDisplayTouchState::Pressed | RobotDisplayTouchState::Held
                    ) =>
                {
                    Some(RobotDisplayTouchStatus {
                        point,
                        state: prev_touch_state.state,
                    })
                }
                SimulatorEvent::KeyDown {
                    keycode: Keycode::Q | Keycode::ESCAPE,
                    ..
                } => {
                    break 'outer;
                }
                SimulatorEvent::Quit => break 'outer,
                _ => continue,
            };
        }

        // Handle touch state
        match (touch_state, prev_touch_state) {
            (Some(status), _) => display.set_touch_status(status),
            (
                None,
                RobotDisplayTouchStatus {
                    point,
                    state: RobotDisplayTouchState::Pressed,
                },
            ) => display.set_touch_status({
                RobotDisplayTouchStatus {
                    point: *point,
                    state: RobotDisplayTouchState::Held,
                }
            }),
            (
                None,
                RobotDisplayTouchStatus {
                    point,
                    state: RobotDisplayTouchState::Released,
                },
            ) => display.set_touch_status({
                RobotDisplayTouchStatus {
                    point: *point,
                    state: RobotDisplayTouchState::Default,
                }
            }),
            _ => (),
        }

        // Draw screen
        display.draw(&mut auton_selector).expect("Infallible");

        window.update(display.target());
    }
}
