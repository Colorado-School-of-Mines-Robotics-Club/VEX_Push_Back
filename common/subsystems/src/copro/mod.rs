use std::{
	cell::RefCell,
	ops::{Deref, DerefMut},
	rc::Rc,
	time::Duration,
};

use coprocessor::{
	requests::{
		CalibrateRequest, CoprocessorRequest, GetPositionRequest, GetVelocityRequest, OtosPosition,
		OtosVelocity, PingRequest, SetOffsetsRequest,
	},
	vexide::CoprocessorSmartPort,
};
use shrewnit::{Degrees, Length, Radians};
use vexide::{controller::ControllerState, smart::SmartPort, time::sleep};

use crate::{ControllableSubsystem, ControllerConfiguration};

pub mod tracking;

pub struct CoproSubsystem {
	port: CoprocessorSmartPort,
	offsets: OtosPosition,
	data: Rc<RefCell<CoproData>>,
}

#[derive(Default)]
pub struct CoproData {
	pub position: OtosPosition,
	pub velocity: OtosVelocity,
	pub forward_travel: Length<f64>,
}

impl CoproSubsystem {
	pub async fn new(port: SmartPort, offsets: OtosPosition) -> Self {
		// Ensure the smart port is actually just a serial device and not a radio or something
		let port = CoprocessorSmartPort::new(port).await;

		let port_clone = port.clone();
		vexide::task::spawn(async move {
			println!("Setting up copro...");
			if let Err(e) = Self::setup(&port_clone, offsets).await {
				eprintln!("Failed to set up copro: {e:?}")
			} else {
				println!("Copro set up!");
			}
		})
		.detach();

		let data = Rc::new(RefCell::new(CoproData::default()));

		let port_clone = port.clone();
		let data_clone = data.clone();
		vexide::task::spawn(Self::background_task(port_clone, data_clone)).detach();

		Self {
			port,
			offsets,
			data,
		}
	}

	pub fn data(&self) -> &Rc<RefCell<CoproData>> {
		&self.data
	}

	pub async fn setup(
		port: &CoprocessorSmartPort,
		offsets: OtosPosition,
	) -> Result<<PingRequest as CoprocessorRequest>::Response, std::io::Error> {
		let ping = port.send_request(PingRequest).await?;

		port.send_request(CalibrateRequest).await?;
		port.send_request(SetOffsetsRequest(offsets)).await?;

		Ok(ping)
	}

	async fn background_task(port: CoprocessorSmartPort, data: Rc<RefCell<CoproData>>) -> ! {
		let mut last_position = OtosPosition::default();

		loop {
			match port.send_request(GetPositionRequest).await {
				Ok(position) => {
					let mut data = data.borrow_mut();

					// Update the position data itself
					data.position = position;

					// Calculate the direction in which we moved, and the total distance that we moved
					let diff = (position.x - last_position.x, position.y - last_position.y);
					let movement_angle =
						f64::atan2(diff.1.canonical(), diff.0.canonical()) * Radians;
					let distance = Length::from_canonical(
						(diff.0.canonical().powi(2) + diff.1.canonical().powi(2)).sqrt(),
					);

					// Use the angle of movement and the heading to find how much we moved in the direction
					// of our current heading (the travel, ignoring any non-"forward" movement)
					let forward_travel_delta = distance
						* f64::cos(
							(movement_angle - (position.heading + 90.0 * Degrees)).to::<Radians>(),
						);

					// Update state and update new forward travel through the sink
					last_position = position;
					data.forward_travel += forward_travel_delta;
				}
				Err(_e) => (), // TODO: error state indication?
			};

			match port.send_request(GetVelocityRequest).await {
				Ok(velocity) => {
					data.borrow_mut().velocity = velocity;
				}
				Err(_e) => (), // TODO: error state indication?
			};

			sleep(Duration::from_millis(3)).await;
		}
	}
}

impl ControllableSubsystem for CoproSubsystem {
	fn direct(&mut self, _state: &ciborium::Value) {}

	fn control(&mut self, controller: &ControllerState, _configuration: ControllerConfiguration) {
		if controller.button_y.is_now_pressed() {
			let port_clone = self.port.clone();
			let offsets = self.offsets;
			vexide::task::spawn(async move {
				match Self::setup(&port_clone, offsets).await {
					Ok(_) => println!("Calibrated!"),
					Err(e) => println!("Unable to calibrate: {e:?}"),
				}
			})
			.detach();
		}
	}
}

impl Deref for CoproSubsystem {
	type Target = CoprocessorSmartPort;

	fn deref(&self) -> &Self::Target {
		&self.port
	}
}

impl DerefMut for CoproSubsystem {
	fn deref_mut(&mut self) -> &mut Self::Target {
		&mut self.port
	}
}
