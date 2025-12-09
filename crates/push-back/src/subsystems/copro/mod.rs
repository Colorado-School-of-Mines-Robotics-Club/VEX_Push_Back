use std::{cell::RefCell, rc::Rc};

use coprocessor::{
    requests::CalibrateRequest,
    vexide::{CoprocessorData, CoprocessorSmartPort},
};
use vexide::{
    controller::ControllerState,
    smart::{SmartDeviceType, SmartPort},
};

use crate::subsystems::{ControllableSubsystem, ControllerConfiguration};

pub mod tracking;

pub struct CoproSubsystem {
    port: CoprocessorSmartPort,
    data: Rc<RefCell<CoprocessorData>>,
}

impl CoproSubsystem {
    pub async fn new(port: SmartPort) -> Self {
        // Ensure the smart port is actually just a serial device and not a radio or something
        assert_eq!(port.device_type(), Some(SmartDeviceType::GenericSerial));

        let (port, data) = CoprocessorSmartPort::new(port).await;
        Self {
            port,
            data: Rc::new(RefCell::new(data)),
        }
    }

    pub fn data(&self) -> Rc<RefCell<CoprocessorData>> {
        self.data.clone()
    }
}

impl ControllableSubsystem for CoproSubsystem {
    fn direct(&mut self, _state: &ciborium::Value) {}

    fn control(&mut self, controller: &ControllerState, _configuration: ControllerConfiguration) {
        if controller.button_y.is_now_pressed() {
            let request_future = self.port.send_request(CalibrateRequest);
            vexide::task::spawn(async move {
                println!("Starting calibration...");
                match request_future.await {
                    Ok(_) => println!("Calibrated!"),
                    Err(e) => println!("Unable to calibrate: {e:?}"),
                }
            })
            .detach();
        }
    }
}
