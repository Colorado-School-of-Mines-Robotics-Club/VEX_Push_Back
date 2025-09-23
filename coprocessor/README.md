# Coprocessor setup

## Magic numbers

- Ethernet gadget MAC addresses
    - Device: `32:98:33:00:00:00`
    - Host:   `32:98:33:00:00:01`
- IP addresses (link-local)
    - IPv4: `169.254.25.0`\*
    - IPv6: `fe80::3098:33ff:fe00:0`

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
- Half-duplex, arbitrated by the host
- Vex brain as host, Pi as device
- Consistent Overhead Byte Stuffing (COBS) for packet framing
- CRC with the following parameters
  | Parameter | Value |
  | - | - |
  | Width | 16 |
  | Polynomial | `0xa2eb` (CRC-16F/4.2) |
  | Initial value | `0xffff` |
  | Reverse input | No |
  | Reverse output | No |
  | XOR output | `0xffff` |
  | Check | `0x624e` |
  | Residue | `0xddfc` |

### Packet format

The format for both request and response packets is as follows (all COBS-encoded):

```
[1 byte ID] [1 byte packet type] [type-specific payload] [2 byte CRC]
```

A response to a request must have the same 1 byte ID as the request.

### Payloads

| Request type    | Request payload | Request Meaning | Response payload | Response meaning        |
| --------------- | --------------- | --------------- | ---------------- | ----------------------- |
| `0x01`          | None            | Calibrate OTOS  | `0xff`           | Successfully calibrated |
| `0x02`          | None            | Request OTOS    | OtosPosition     | Data on the current position from OTOS |

### Structs

All structs are little-endian

#### OtosPosition

Values are returned as a signed 16-bit integer representing a portion within some range.

```c
    // Range: [-10m, 10m]
    int16_t x;
    // Range: [-10m, 10m]
    int16_t y;
    // Range: [-180deg, 180deg]
    int16_t h;
```
