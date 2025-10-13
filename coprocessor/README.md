# Coprocessor setup

## Parts (per bot)

- https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/
- https://www.pishop.us/product/dc-dc-ldo-5v-dc-dc-converter-5v-1-2a-output-6-5v-to-15v-dc-input/
- https://www.pishop.us/product/max-485-module-rs-485-module-ttl-to-rs-485-converter-module/
- Sufficient wiring

## Wiring

### MAX 485 converter

The converter has 4 main pins that must be connected to the Raspberry PI:

| Converter pin | Purpose              | Destination |
| ------------- | -------------------- | ----------- |
| RO            | RS485 Recieve        | RPI UART RX (GPIO 14, Board 8) |
| DI            | RS485 Send           | RPI UART TX (GPIO 15, Board 10) |
| RE            | Recieve enable (low) | RPI GPIO 17 (Board 11) |
| DE            | Driver enable (high) | RPI GPIO 18 (Board 12) |

The PI should pull both the DE and RE pins low by default to enable reciever mode,
and wait for a request from the brain. Once a request has been recieved and validated,
the PI should then pull both pins high to enable driver mode, and send the response.
After the response has been fully sent, reciever mode should be enabled again.

## Communication protocol

- 921600 baud UART
- Half-duplex
    - All communication is request -> response, with the brain requesting
- Consistent Overhead Byte Stuffing (COBS) for packet framing
- Where possible, data is forwarded from sensors with no procesing on the pi

## Code

## Vex Brain

The code that runs on the brain is written as a rust crate to be depended on by
the main robot code. It takes a given smart port as a UART serial interface and
uses that for communication. This side of the code does the majority of the
processing, including convering raw data from the sensor to reasonable units, as
the brain is more suited for real-time processing.

## Pi (OS)

The pi uses NixOS to make it easy to automate initial Pi setup and ensure
reproducibility. To create an SD image for use on the pi, use the command `nix
build '.#nixosConfigurations.rpi<N>.config.system.build.sdImage' -L`, where
`<N>` is replaced with the numeric identifier for the Pi (this may require
remote builders or binfmt). Then, after the image is constructed, write the
resulting image file to an SD card, and insert it into a Pi. The Pi should then
boot properly and have everything configured out of the box. In order for the
actual coproccessor communication script to run, it must first be uploaded over
SSH (the left Micro USB port is configured as an ethernet gadget, and will allow
link-local communication and mDNS usage) to `/home/pi/copro.py`. The systemd
service `copro.service` will automatically run this file at boot, and can be
managed manually over SSH.

## Pi (communication)

The code that actually does communication with the brain is written as a python
script that uses a linux serial inferface to communicate with the brain, and i2c
to interact with sensors. Minimal processing should be done on any data being
handled by this script, as python is not designed for speed and the script is
not the only code running on the pi.

<!--
TODO: https://documentation.ubuntu.com/real-time/latest/how-to/isolate-workload-cpusets/
Give all normal slices 3/4 of the cores, and make a new slice for the script with it's own core

This would allow providing an isolated core for the python program to run on,
ensuring script execution is never delayed due to shared CPU use.
-->
