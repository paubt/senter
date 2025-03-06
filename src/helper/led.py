#  Blink an LED with the LGPIO library
#  Uses lgpio library, compatible with kernel 5.11
#  Author: William 'jawn-smith' Wilson

import time
import lgpio

LED = 24

# open the gpio chip and set the LED pin as output
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, LED)

try:
    while True:
        # Turn the GPIO pin on
        print("1")
        lgpio.gpio_write(h, LED, 1)
        print("2")
        time.sleep(1)
        print("3")
        # Turn the GPIO pin off
        lgpio.gpio_write(h, LED, 0)
        time.sleep(1)
except KeyboardInterrupt:
    lgpio.gpio_write(h, LED, 0)
    lgpio.gpiochip_close(h)
