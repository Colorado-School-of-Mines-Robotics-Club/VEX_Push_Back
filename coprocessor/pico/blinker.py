from typing import Tuple

import rp2
from pio.blinker import (
    blinker,
    blinker_FREQUENCY,
)

MAX_BLINK_COUNT = 2**32 - 1


class PioBlinker:
    def __init__(self, pio_idx: Tuple[int, int], pin: machine.Pin):
        """Initializes the blinker

        :param pin: The pin an LED is controlled by
        """
        self.pin = pin
        self.sm = rp2.StateMachine(
            pio_idx[0] * 4 + pio_idx[1],
            blinker,
            freq=blinker_FREQUENCY,
            sideset_base=pin,
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
            self.sm.put(count - 1)
            self.sm.active(1)
