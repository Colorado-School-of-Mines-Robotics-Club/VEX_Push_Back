#![no_std]
#![allow(deprecated)]

extern crate alloc;

use core::{
	cell::{Cell, RefCell},
	hash::{Hasher, SipHasher},
	mem::MaybeUninit,
	sync::atomic::AtomicU8,
};

use alloc::boxed::Box;
use rand_pcg::Pcg32;
use vex_sdk::*;
use vexide::task::task_local;

task_local! {
	static HASHER: RefCell<SipHasher> = RefCell::new(SipHasher::default());
	static PRNG: RefCell<Pcg32> = RefCell::new(Pcg32::new(0xcafef00dd15ea5e5, 0xa02bdbf7bb3c0a7));
	static LAST_SEED: Cell<Option<u32>> = Cell::new(None);
}

#[cfg(getrandom_backend = "extern_impl")]
#[getrandom::implementation::fill_uninit]
unsafe extern "Rust" fn vexide_getrandom_fill_uninit(
	dest: &mut [MaybeUninit<u8>],
) -> Result<(), getrandom::Error> {
	use core::time::Duration;

	// Reseed the PRNG if it hasn't yet or has been 2s since last seed
	let current_time = unsafe { vexSystemTimeGet() };
	if LAST_SEED.with(|v| v.get().is_none_or(|i| current_time - i > 2_000)) {
		let seed = HASHER.with_borrow_mut(|hasher| {
			seed_hasher(hasher);
			hasher.finish()
		});
		PRNG.with(|prng| prng.replace(Pcg32::seed_from_u64(seed)));
		LAST_SEED.with(|last_seed| last_seed.replace(Some(current_time)));
	}
	PRNG.with_borrow_mut(|prng| {
		let mut value;
		for (i, dest_byte) in dest.iter_mut().enumerate() {
			if i % 4 == 0 {
				value = prng.next_u32();
			}
			dest_byte.write((value & 0xFF) as _);
			value >>= 8;
		}
	});

	Ok(())
}

macro_rules! hash_sdk_output {
	($hasher:ident, $call:expr, $out_name:ident: $out_type:ty) => {
		// Pointless overoptimization GO
		let out: [u8; size_of::<$out_type>()] = unsafe {
			let mut __data = MaybeUninit::<$out_type>::uninit();
			let $out_name = __data.as_mut_ptr();
			$call;
			// SAFETY: Theoretically always safe, it is only one-way and u8 arrays are always 1-aligned
			core::mem::transmute(__data.assume_init())
		};
		$hasher.write(&out);
	};
}

fn hash_adi(expander: V5_DeviceT, port: u32, hasher: &mut impl Hasher) {
	hasher.write_u8(unsafe { vexDeviceAdiPortConfigGet(expander, port - 1) }.0);
	hasher.write_i32(unsafe { vexDeviceAdiValueGet(expander, port - 1) });
}

fn hash_smart(port: V5_DeviceT, device_type: V5_DeviceType, hasher: &mut impl Hasher) {
	if device_type != V5_DeviceType::kDeviceTypeNoSensor {
		hasher.write_u8(1); // Connected
		hasher.write_u32(unsafe { vexDeviceGetTimestamp(port) });
		hasher.write_u8(device_type.0);

		match device_type {
			V5_DeviceType::kDeviceTypeMotorSensor => {
				hasher.write(&unsafe { vexDeviceMotorTemperatureGet(port) }.to_ne_bytes());
				hasher.write(
					&unsafe { vexDeviceMotorPositionRawGet(port, core::ptr::null_mut()) }
						.to_ne_bytes(),
				);
			}
			V5_DeviceType::kDeviceTypeAbsEncSensor => {
				hasher.write_i32(unsafe { vexDeviceAbsEncAngleGet(port) });
				hasher.write_i32(unsafe { vexDeviceAbsEncVelocityGet(port) });
			}
			V5_DeviceType::kDeviceTypeImuSensor => {
				hash_sdk_output!(hasher, vexDeviceImuRawGyroGet(port, raw), raw: V5_DeviceImuRaw);

				// if let Ok(physical_orientation) = imu.physical_orientation() {
				// 	hasher.write_u8(physical_orientation as u8);
				// }
				// if let Ok(heading) = imu.heading() {
				// 	hasher.write(&heading.as_degrees().to_ne_bytes());
				// }
			}
			V5_DeviceType::kDeviceTypeDistanceSensor => {
				let distance = unsafe { vexDeviceDistanceDistanceGet(port) };

				if distance != 9999 {
					hasher.write_u32(distance);
					hasher.write_i32(unsafe { vexDeviceDistanceObjectSizeGet(port) });
					hasher
						.write(&unsafe { vexDeviceDistanceObjectVelocityGet(port) }.to_ne_bytes());
					hasher.write_u32(unsafe { vexDeviceDistanceConfidenceGet(port) });
				} else {
					hasher.write_u8(0);
				}
			}
			V5_DeviceType::kDeviceTypeVisionSensor => (),
			V5_DeviceType::kDeviceTypeAiVisionSensor => (),
			V5_DeviceType::kDeviceTypeMagnetSensor => {
				hasher.write(&unsafe { vexDeviceMagnetCurrentGet(port) }.to_ne_bytes());
				hasher.write_i32(unsafe { vexDeviceMagnetPowerGet(port) });
				hasher.write_u32(unsafe { vexDeviceMagnetStatusGet(port) });
				hasher.write(&unsafe { vexDeviceMagnetTemperatureGet(port) }.to_ne_bytes());
			}
			V5_DeviceType::kDeviceTypeLightTowerSensor => {
				hasher.write_u32(unsafe { vexDeviceLightTowerRgbGet(port) });
			}
			V5_DeviceType::kDeviceTypeArmDevice => (),
			V5_DeviceType::kDeviceTypeOpticalSensor => {
				hash_sdk_output!(hasher, vexDeviceOpticalRawGet(port, raw), raw: V5_DeviceOpticalRaw);
				hasher.write(&unsafe { vexDeviceOpticalProximityGet(port) }.to_ne_bytes());
			}
			V5_DeviceType::kDeviceTypeGpsSensor => {
				hash_sdk_output!(hasher, vexDeviceGpsAttitudeGet(port, attitude, false), attitude: V5_DeviceGpsAttitude);
				hash_sdk_output!(hasher, vexDeviceGpsRawGyroGet(port, raw), raw: V5_DeviceGpsRaw);
				hash_sdk_output!(hasher, vexDeviceGpsRawAccelGet(port, raw), raw: V5_DeviceGpsRaw);
			}
			V5_DeviceType::kDeviceTypeRadioSensor => (),
			V5_DeviceType::kDeviceTypeAdiSensor => {
				for adi_port_num in 1..=8 {
					hash_adi(port, adi_port_num, hasher);
				}
			}
			V5_DeviceType::kDeviceTypeGenericSerial => {
				hasher.write_i32(unsafe { vexDeviceGenericSerialPeekChar(port) });
				hasher.write_i32(unsafe { vexDeviceGenericSerialReceiveAvail(port) });
				hasher.write_i32(unsafe { vexDeviceGenericSerialWriteFree(port) });
			}
			_ => (),
		};
	} else {
		hasher.write_u8(0); // Disconnected
	}
}

fn hash_controller(hasher: &mut impl Hasher, controller: V5_ControllerId) {
	let connection = unsafe { vexControllerConnectionStatusGet(controller) };
	hasher.write_u8(connection.0);
	if connection != V5_ControllerStatus::kV5ControllerOffline {
		for index in (0..=22).map(V5_ControllerIndex) {
			hasher.write_i32(unsafe { vexControllerGet(controller, index) });
		}
	}
}

// TODO make hash_state or smth that takes a &mut impl Hasher and inputs everything
//
// that way we can have a global program SipHasher that occasionally gets re-seeded
// with the new state of things (maybe in a very very slow-running task)
//
// also that means we can actually use motor current and voltage and stuff because
// theoretically it gets used throughout the program!!!! yipee

/// Feeds many known sources of entropy into a passed [Hasher], effectively allowing
/// it to then be finished and used as a random value.
///
/// This uses the following entropy sources:
/// - Count of times this function has been called
/// - Stack & heap allocation positions
/// - Battery capacity, voltage, current, temp
/// - Competition state
/// - System version
/// - Time since program & brain start
/// - State & configuration of all ADI ports
/// - Readings from most common smart port devices, such as motor encoder position,
///   distance readings, or IMU readings
/// - Primary & partner controller state
pub fn seed_hasher(hasher: &mut impl Hasher) {
	static INCREMENTOR: AtomicU8 = AtomicU8::new(0);
	INCREMENTOR.fetch_add(1, core::sync::atomic::Ordering::Relaxed);

	// Use stack allocation and heap allocation locations (what rust std uses when OS rng not availible)
	{
		let stack = 0;
		let heap = Box::new(0);

		hasher.write_usize(core::ptr::from_ref(&stack).addr());
		hasher.write_usize(core::ptr::from_ref(&*heap).addr());
	}

	// Use the amount of times this function has been called
	hasher.write_u8(INCREMENTOR.load(core::sync::atomic::Ordering::Relaxed));

	// Use battery state
	hasher.write(&unsafe { vexBatteryTemperatureGet() }.to_ne_bytes());
	hasher.write(&unsafe { vexBatteryCapacityGet() }.to_ne_bytes());
	hasher.write_i32(unsafe { vexBatteryCurrentGet() });
	hasher.write_i32(unsafe { vexBatteryVoltageGet() });

	// Use competition state
	hasher.write_u32(unsafe { vexCompetitionStatus() });

	// Use VexOS version
	hasher.write_u32(unsafe { vexSystemVersion() });

	// Use how long the brain & program have been running for
	hasher.write_u64(unsafe { vexSystemPowerupTimeGet() });
	hasher.write_u64(unsafe { vexSystemHighResTimeGet() });

	// Use state of all ADI ports TODO: fold into following because internal adi is just an adi expander on port 22
	for port_num in 1..=8 {
		hash_adi(unsafe { vexDeviceGetByIndex(21) }, port_num, hasher);
	}

	// Use state of all smart ports
	let mut device_types: [V5_DeviceType; V5_MAX_DEVICE_PORTS] = unsafe { core::mem::zeroed() };
	_ = unsafe { vexDeviceGetStatus(device_types.as_mut_ptr()) };
	for port_num in 1..=21 {
		let port = unsafe { vexDeviceGetByIndex(port_num - 1) };
		hash_smart(port, device_types[port_num as usize - 1], hasher);
	}

	// Use controller state
	hash_controller(hasher, V5_ControllerId::kControllerMaster);
	hash_controller(hasher, V5_ControllerId::kControllerPartner);
}
