use std::time::{Duration, Instant};

use coprocessor::{
	requests::{GetPositionRequest, PingRequest, RAINBOW_ROTATE, RAINBOW_STATIC, SetLedsRequest},
	vexide::CoprocessorSmartPort,
};
use vexide::prelude::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
	let coprocessor = CoprocessorSmartPort::new(peripherals.port_6).await;

	_ = coprocessor
		.send_request(SetLedsRequest::<{ RAINBOW_STATIC }>)
		.await;
	sleep(Duration::from_secs(1)).await;
	_ = coprocessor.send_request(SetLedsRequest::<0xFF0000>).await;
	sleep(Duration::from_secs(1)).await;
	_ = coprocessor.send_request(SetLedsRequest::<0x00FF00>).await;
	sleep(Duration::from_secs(1)).await;
	_ = coprocessor.send_request(SetLedsRequest::<0x0000FF>).await;
	sleep(Duration::from_secs(1)).await;
	_ = coprocessor
		.send_request(SetLedsRequest::<{ RAINBOW_ROTATE }>)
		.await;
	sleep(Duration::from_secs(1)).await;
}
