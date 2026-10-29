# SPDX-FileCopyrightText: Copyright (c) 2026 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""Timestamp button presses with the EVI button
or pin to the hundredth of a second."""

import time

import board

import adafruit_rv8803

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
rtc = adafruit_rv8803.RV8803(i2c)

# EVI button when pressed -> GND pulls EVI low, so trigger on the falling edge
rtc.event_rising_edge = False
# Filter out short glitches such as switch bounce
rtc.event_filter = adafruit_rv8803.EVENT_FILTER_16MS
# Keep the clock running normally when an event happens
rtc.event_reset = False
# Save the seconds and hundredths when an event happens
rtc.event_capture_enabled = True

# Clear any old event before waiting for a new one
rtc.event_status = False

print("Press the EVI button")

while True:
    if rtc.event_status:
        seconds, hundredths = rtc.event_timestamp
        t = rtc.datetime
        print(f"Event at {t.tm_hour}:{t.tm_min:02}:{seconds:02}.{hundredths:02}")
        rtc.event_status = False
    time.sleep(0.01)
