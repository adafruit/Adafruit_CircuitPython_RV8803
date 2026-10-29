# SPDX-FileCopyrightText: Copyright (c) 2026 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""Run the countdown timer so it fires every 5 seconds."""

import time

import board

import adafruit_rv8803

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
rtc = adafruit_rv8803.RV8803(i2c)

# Count 5 ticks of a 1 Hz clock. Setting these stops the timer.
rtc.timer_clock = adafruit_rv8803.TIMER_1HZ
rtc.timer_value = 5

# Clear any old timer event, then start counting
rtc.timer_status = False
rtc.timer_enabled = True
# Optional: pull the INT pin low each time the timer fires
# rtc.timer_interrupt = True

start = time.monotonic()
print("Timer started")

while True:
    if rtc.timer_status:
        # The timer reloads itself, so it keeps firing until timer_enabled is False
        print(f"Timer fired at {time.monotonic() - start:.1f} seconds")
        rtc.timer_status = False
    time.sleep(0.1)
