def decode(buffer: bytes) -> bytes:
    next_null = buffer[0]
    next_is_overhead = next_null == 255
    out = bytearray()

    for byte in buffer[1:]:
        if byte == 0:
            break

        next_null -= 1
        if next_null == 0:
            if not next_is_overhead:
                out.append(0)
            next_null = byte
            next_is_overhead = next_null == 255
        else:
            out.append(byte)

    return bytes(out)

def encode(buffer: bytes) -> bytes:
    out = bytearray([0x00])
    last_null_pos = 0

    for byte in buffer:
        if (len(out) - last_null_pos) == 0xFF:
            out[last_null_pos] = 0xFF
            last_null_pos = len(out)
            out.append(0x00)

        out.append(byte)
        if byte == 0x00:
            out[last_null_pos] = len(out) - 1 - last_null_pos
            last_null_pos = len(out) - 1

    out[last_null_pos] = len(out) - last_null_pos
    out.append(0x00)

    return bytes(out)


cobs_tests = [
    # Thanks wikipedia
    ([0x00], [0x01, 0x01, 0x00]),
    ([0x00, 0x00], [0x01, 0x01, 0x01, 0x00]),
    ([0x00, 0x11, 0x00], [0x01, 0x02, 0x11, 0x01, 0x00]),
    ([0x11, 0x22, 0x00, 0x33], [0x03, 0x11, 0x22, 0x02, 0x33, 0x00]),
    ([0x11, 0x22, 0x33, 0x44], [0x05, 0x11, 0x22, 0x33, 0x44, 0x00]),
    ([0x11, 0x00, 0x00, 0x00], [0x02, 0x11, 0x01, 0x01, 0x01, 0x00]),
    ([b for b in range(0x01, 0xFF)], [0xFF] + [b for b in range(0x01, 0xFF)] + [0x00]),
    ([b for b in range(0x00, 0xFF)], [0x01, 0xFF] + [b for b in range(0x01, 0xFF)] + [0x00]),
    ([b for b in range(0x01, 0xFF + 1)], [0xFF] + [b for b in range(0x01, 0xFF)] + [0x02, 0xFF, 0x00]),
    ([b for b in range(0x02, 0xFF + 1)] + [0x00], [0xFF] + [b for b in range(0x02, 0xFF + 1)] + [0x01, 0x01, 0x00]),
    ([b for b in range(0x03, 0xFF + 1)] + [0x00, 0x01], [0xFE] + [b for b in range(0x03, 0xFF + 1)] + [0x02, 0x01, 0x00]),
]

if __name__ == "__main__":
    for decoded, encoded in cobs_tests:
        # Check decoding
        test_encode = encode(bytes(decoded))
        if test_encode != bytes(encoded):
            print(f"Failed to encode: {decoded}")
            print(f"Expected: {encoded}")
            print(f"Actual: {list(test_encode)}")
            print()

        # Check decoding
        test_decode = decode(bytes(encoded))
        if test_decode != bytes(decoded):
            print(f"Failed to decode: {encoded}")
            print(f"Expected: {decoded}")
            print(f"Actual: {list(test_decode)}")
            print()
