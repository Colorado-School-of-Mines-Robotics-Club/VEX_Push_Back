#!/usr/bin/env python3
import qwiic_otos
import serial
import time
import struct
import os
import re
from cobs import cobs
from enum import IntEnum
from typing import List

class Constants:
	# V5 Brain
	BRAIN_BAUD_RATE = 115200
	BRAIN_TIMEOUT = 1
	BRAIN_DEVICE_REGEX = r"^usb-VEX_Robotics__Inc_VEX_Robotics_V5_Brain_-_[A-F0-9]{8}-if02$"

	# Control codes sent from V5 Brain
	class Codes(IntEnum):
			KEEPALIVE = 0x00
			RECALIBRATE = 0x01

	# Sensor
	LINEAR_SCALAR = 1.0
	ANGULAR_SCALAR = 1.0
	OFFSET_X = 0
	OFFSET_Y = 0
	OFFSET_ANGLE = 0

	# Result enums
	class RecvResult(IntEnum):
		SUCCESS = 0x00
		DISCONNECTED = 0x01

	class SendResult(IntEnum):
		SUCCESS = 0x00
		OTOS_DISCONNECTED = 0x01
		BRAIN_DISCONNECTED = 0x02

class OtosProgram:
	otos_sensor: qwiic_otos.QwiicOTOS | None = None
	brain_connection: serial.Serial | None = None
	last_keepalive: float | None = None
	recv_buffer: List[int] = []

	def recalibrate_otos(self, otos_sensor: qwiic_otos.QwiicOTOS):
		# Reset all settings
		otos_sensor.begin()

		# Calibrate sensor
		otos_sensor.calibrateImu()

		# Configure sensor
		otos_sensor.setLinearScalar(1.0)
		otos_sensor.setAngularScalar(1.0)

		offset = qwiic_otos.Pose2D(Constants.OFFSET_X, Constants.OFFSET_Y, Constants.OFFSET_ANGLE)
		otos_sensor.setOffset(offset)

		# Reset sensor to origin
		otos_sensor.resetTracking()

	def wait_otos(self):
		i = 0
		print("Waiting for OTOS sensor connection...")
		while True:
			otos_sensor = qwiic_otos.QwiicOTOS()
			if otos_sensor.is_connected():
				# Initialize sensor
				self.recalibrate_otos(otos_sensor)

				# Return sensor class
				print("OTOS sensor connected and calibrated!")
				self.otos_sensor = otos_sensor
				return
			else:
				time.sleep(1)
				i += 1
				if i % 30 == 0:
					print(f"! Unable to connect to OTOS sensor after {i}s")
				continue

	def wait_brain(self):
		i = 0
		print("Waiting for v5 brain connection...")
		while True:
			# Get all linux serial devices
			try:
				serial_devices = os.listdir("/dev/serial/by-id/")
			except:
				i += 1
				if i % 30 == 0:
					print(f"! Unable to connect to brain after {i}s")
				time.sleep(1)
				continue

			matching_serial_devices = list(filter(
				lambda dev: re.search(Constants.BRAIN_DEVICE_REGEX, dev) != None,
				serial_devices
			))

			# If we don't have exactly one matching device, wait longer
			if len(matching_serial_devices) != 1:
				i += 1
				if i % 30 == 0:
					print(f"! Unable to connect to brain after {i}s")
				time.sleep(1)
				continue

			# Setup serial object
			serial_connection = serial.Serial()
			serial_connection.baudrate = Constants.BRAIN_BAUD_RATE
			serial_connection.timeout = Constants.BRAIN_TIMEOUT
			serial_connection.port = f"/dev/serial/by-id/{matching_serial_devices[0]}"

			# Attempt to connect
			try:
				serial_connection.open()
				self.brain_connection = serial_connection
				print("Brain connection opened!")
				return
			except:
				i += 1
				if i % 30 == 0:
					print(f"! Unable to connect to brain after {i}s")
				time.sleep(1)
				continue

	def recv_data(self) -> Constants.RecvResult:
		if self.brain_connection == None:
			return Constants.RecvResult.DISCONNECTED

		try:
			data = self.brain_connection.read_all()
			if data != None:
				self.recv_buffer.extend(data)

			self.flush_buffer()
			return Constants.RecvResult.SUCCESS
		except Exception as e:
			print(e)
			return Constants.RecvResult.DISCONNECTED

	def flush_buffer(self):
		for [i, byte] in enumerate(self.recv_buffer):
			if byte == 0x00: # COBS packet boundary
				decoded = cobs.decode(bytes(self.recv_buffer[0:i]))
				self.recv_buffer = self.recv_buffer[i+1:] # Remove this packet from buffer
				self.handle_packet(decoded)

	def handle_packet(self, data):
		if data.startswith(b"sout"):
			for line in str(data[4:]).splitlines():
				print(f"[STDOUT] {line}")
		elif data.startswith(b"serr"):
			match data[4]:
				case Constants.Codes.KEEPALIVE:
					self.last_keepalive = time.time()
				case Constants.Codes.RECALIBRATE:
					if self.otos_sensor != None:
						self.recalibrate_otos(self.otos_sensor)

	def send_latest_pos(self) -> Constants.SendResult:
		if self.otos_sensor == None:
			return Constants.SendResult.OTOS_DISCONNECTED

		try:
			position = self.otos_sensor.getPosition()
		except:
			return Constants.SendResult.OTOS_DISCONNECTED

		if self.brain_connection == None:
			return Constants.SendResult.BRAIN_DISCONNECTED

		packed = struct.pack("<ddd", position.x, position.y, position.h)
		encoded = cobs.encode(packed)

		try:
			bytes_written = self.brain_connection.write(encoded)
			zero_written = self.brain_connection.write(b'\0')

			if bytes_written != len(encoded) or zero_written != 1:
				print("WARNING: Wrong amount of bytes written to serial!")
		except:
			return Constants.SendResult.BRAIN_DISCONNECTED

		return Constants.SendResult.SUCCESS

	def main(self):
		print("Setting up...")

		self.wait_otos()
		self.wait_brain()

		print("Running main loop...")
		while True:
			# Recieve any data from the brain
			match self.recv_data():
				case Constants.RecvResult.SUCCESS:
					pass
				case Constants.RecvResult.DISCONNECTED:
					self.wait_brain()

			# Get latest sensor data and send to brain
			match self.send_latest_pos():
				case Constants.SendResult.SUCCESS:
					pass
				case Constants.SendResult.BRAIN_DISCONNECTED:
					self.wait_brain()
				case Constants.SendResult.OTOS_DISCONNECTED:
					self.wait_otos()

if __name__ == "__main__":
	program = OtosProgram()
	program.main()
