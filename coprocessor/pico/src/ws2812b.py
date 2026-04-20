from typing import Final
import machine

import rp2
from pio.ws2812b import ws2812, ws2812_T1, ws2812_T2, ws2812_T3  # pyright: ignore[reportUnknownVariableType]

DATA_RATE = 800_000  # Kbps


class PioWS2812B:
    pio_idx: Final[tuple[int, int]]
    sm: Final[rp2.StateMachine]
    buffer: Final[bytearray]
    dma: Final[rp2.DMA]

    def __init__(
        self,
        pio_idx: tuple[int, int],
        pin: machine.Pin,
        led_count: int,
    ):
        """Sets up a PIO state machine for WS2812b LED usage

        :param pio: A tuple of (pio, state_machine) indexes to use. Ensure this specific index pair is not in use.
        :param pin: The pin to use for WS2812b output
        :param led_count: The number of LEDs to control
        """

        # Save the PIO & state machine index for future DMA usage
        self.pio_idx = pio_idx
        # Construct and start the PIO state machine
        self.sm = rp2.StateMachine(
            pio_idx[0] * 4 + pio_idx[1],
            ws2812,
            freq=(ws2812_T1 + ws2812_T2 + ws2812_T3) * DATA_RATE,
            sideset_base=pin,
        )
        self.sm.active(1)
        # Allocate a buffer for storing pixel data
        self.buffer = bytearray(4 * led_count)
        # Reserve a DMA channel for data transfers to the PIO
        self.dma = rp2.DMA()

    # Allow ws2812b[i] = (r, g, b)
    def __setitem__(self, i: int, rgb: tuple[int, int, int]):
        self.buffer[i * 4 + 3] = rgb[1]  # G
        self.buffer[i * 4 + 2] = rgb[0]  # R
        self.buffer[i * 4 + 1] = rgb[2]  # B

    # Allow ws2812b[i] -> (r, g, b)
    def __getitem__(self, i: int) -> tuple[int, int, int]:
        return (self.buffer[i * 4 + 2], self.buffer[i * 4 + 3], self.buffer[i * 4 + 1])

    def write(self):
        """Writes stored data to the LED strip. Ensure this is only called with pauses of 50us or more to avoid accidental chaining."""

        if self.dma.active() != 0:
            return  # Ignore write if one is currently in progress

        # Copy packed GRB data to the PIO state machine's TX FIFO using DMA
        dma_ctrl = self.dma.pack_ctrl(
            size=2,  # Transfer bytes, 24 bit values don't fit nicely into word/half-word increments
            inc_read=True,
            inc_write=False,  # Always write to the same position, the start of the PIO FIFO
            # Use the correct state machine's TX FIFO data request for writing
            treq_sel=(self.pio_idx[0] << 3) + self.pio_idx[1],
        )
        self.dma.config(
            read=self.buffer,
            write=self.sm,
            count=len(self.buffer),
            ctrl=dma_ctrl,
            trigger=True,
        )
