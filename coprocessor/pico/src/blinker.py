from typing import Final
import rp2
from pio.blinker import (
    blinker, # pyright: ignore[reportUnknownVariableType]
    blinker_FREQUENCY,
)
import machine

MAX_BLINK_COUNT = 2**32 - 1


class PioBlinker:
    pin: Final[machine.Pin]
    sm: Final[rp2.StateMachine]

    def __init__(self, pio_idx: tuple[int, int], pin: machine.Pin):
        """Initializes the blinker

        :param pin: The pin an LED is controlled by
        """
        self.pin = pin
        self.sm = rp2.StateMachine(
            pio_idx[0] * 4 + pio_idx[1],
            blinker,
            freq=blinker_FREQUENCY,
            set_base=pin,
            in_shiftdir=rp2.PIO.SHIFT_RIGHT,
        )
        self.sm.active(0)

    def blink(
        self,
        count: int,
    ):
        """Starts blinking the LED at a specified rate.

        The change occurs after the current blink cycle and delay are up.
        If 0 is passed, the LED will be solid."""
        if count > MAX_BLINK_COUNT:
            count = MAX_BLINK_COUNT
        elif count < 1:
            self.sm.active(0)
            self.pin.on()
        else:
            self.sm.active(1)
            self.sm.restart()
            self.sm.put(count - 1)
