WS2812B_PIO = (0, 0)
BLINKER_PIO = (1, 0)

RS485_EN_PIN   = const("GP11")
RS485_TX_PIN   = const("GP12")
RS485_RX_PIN   = const("GP13")
ADDR_LED_PIN   = const("GP10")
OTOS_SDA_PIN   = const("GP8")
OTOS_SCL_PIN   = const("GP9")
STATUS_LED_PIN = const("GP2")

REQ_GET_POSITION     = const(112) # p
REQ_SET_POSITION     = const(80) # P
REQ_GET_VELOCITY     = const(118) # v
REQ_CALIBRATE        = const(99) # c
REQ_SET_OFFSETS      = const(111) # o
REQ_SET_SCALARS      = const(115) # s
REQ_GET_STDDEV       = const(83) # S
REQ_PING             = const(97) # a
REQ_SET_LEDS         = const(108) # l

LEDS_RAINBOW_STATIC = const(0xFF000001)
LEDS_RAINBOW_ROTATE = const(0xFF000002)
