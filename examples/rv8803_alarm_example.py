# SPDX-FileCopyrightText: Copyright (c) 2026 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""Set an alarm for the start of the next minute and wait for it to fire."""

import time

import board

import adafruit_rv8803

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
rtc = adafruit_rv8803.RV8803(i2c)

# Turn off the INT pin while the alarm is being changed
rtc.alarm_interrupt = False

# Only tm_hour, tm_min and the day fields are used by the alarm
alarm_minute = (rtc.datetime.tm_min + 1) % 60
alarm_time = time.struct_time((2000, 1, 1, 0, alarm_minute, 0, 0, -1, -1))

# ALARM_MINUTE matches the minute only, so this fires once an hour
rtc.alarm = (alarm_time, adafruit_rv8803.ALARM_MINUTE)
# To fire every day at 7:30 instead, match the hour and minute:
# alarm_time = time.struct_time((2000, 1, 1, 7, 30, 0, 0, -1, -1))
# rtc.alarm = (alarm_time, adafruit_rv8803.ALARM_HOUR_MIN)

# Clear any old alarm before waiting for a new one
rtc.alarm_status = False
# Optional: pull the INT pin low when the alarm fires
# rtc.alarm_interrupt = True

print(f"Alarm set for minute {alarm_minute:02} of every hour")

while True:
    t = rtc.datetime
    if rtc.alarm_status:
        print(f"Alarm! It is {t.tm_hour}:{t.tm_min:02}:{t.tm_sec:02}")
        rtc.alarm_status = False
    else:
        print(f"Waiting... {t.tm_hour}:{t.tm_min:02}:{t.tm_sec:02}")
    time.sleep(1)
