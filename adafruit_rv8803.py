# SPDX-FileCopyrightText: Copyright (c) 2026 Liz Clark for Adafruit Industries
#
# SPDX-License-Identifier: MIT
"""
`adafruit_rv8803`
================================================================================

CircuitPython driver for the Adafruit RV-8803 High Accuracy Real Time Clock Breakout Board


* Author(s): Liz Clark

Implementation Notes
--------------------

**Hardware:**

* `Adafruit RV-8803 Real Time Clock Breakout Board <https://www.adafruit.com/product/6538>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time

from adafruit_bus_device.i2c_device import I2CDevice
from adafruit_register.i2c_bit import ROBit, RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_register.i2c_struct import ROUnaryStruct, Struct, UnaryStruct
from micropython import const

try:
    from busio import I2C
except ImportError:
    pass

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_RV8803.git"

_DEFAULT_ADDR = const(0x32)

# Registers
_SECONDS = const(0x00)
_RAM = const(0x07)
_ALARM_MIN = const(0x08)
_ALARM_HOUR = const(0x09)
_ALARM_DAY = const(0x0A)
_TIMER0 = const(0x0B)
_EXT = const(0x0D)
_FLAG = const(0x0E)
_CTRL = const(0x0F)
_HUNDREDTHS = const(0x10)
_HUNDREDTHS_CP = const(0x20)
_OFFSET = const(0x2C)
_EVCTRL = const(0x2F)

# Flag register bits
_UF = const(0x20)
_TF = const(0x10)
_AF = const(0x08)
_EVF = const(0x04)
_V2F = const(0x02)
_V1F = const(0x01)
_FLAG_MASK = const(0x3F)

# Countdown timer clock sources
TIMER_4096HZ = const(0x00)  # 244.14 us/tick, max ~1 s
TIMER_64HZ = const(0x01)  # 15.625 ms/tick, max ~64 s
TIMER_1HZ = const(0x02)  # 1 s/tick, max ~68 min
TIMER_1_60HZ = const(0x03)  # 60 s/tick, max ~68 hr

# CLKOUT frequencies
CLKOUT_32KHZ = const(0x00)
CLKOUT_1KHZ = const(0x01)
CLKOUT_1HZ = const(0x02)

# Periodic update modes
UPDATE_SECOND = const(0x00)
UPDATE_MINUTE = const(0x01)

# External event filter times
EVENT_FILTER_NONE = const(0x00)
EVENT_FILTER_4MS = const(0x01)
EVENT_FILTER_16MS = const(0x02)
EVENT_FILTER_125MS = const(0x03)

# Alarm modes: bit 0 ignores minutes, bit 1 ignores hours, bit 2 ignores day
ALARM_MIN_HOUR_DAY = const(0x00)
ALARM_HOUR_DAY = const(0x01)
ALARM_MIN_DAY = const(0x02)
ALARM_DAY = const(0x03)
ALARM_HOUR_MIN = const(0x04)
ALARM_HOUR = const(0x05)
ALARM_MINUTE = const(0x06)
ALARM_EVERY_MINUTE = const(0x07)

# Weekday bits for alarm_weekdays (OR together to match several days)
SUNDAY = const(0x01)
MONDAY = const(0x02)
TUESDAY = const(0x04)
WEDNESDAY = const(0x08)
THURSDAY = const(0x10)
FRIDAY = const(0x20)
SATURDAY = const(0x40)

def _bcd2bin(value: int) -> int:
    return value - 6 * (value >> 4)

def _bin2bcd(value: int) -> int:
    return value + 6 * (value // 10)

def _check_bcd(value: int, maximum: int) -> int:
    if (value & 0x0F) > 9 or _bcd2bin(value) > maximum:
        raise RuntimeError("Invalid BCD data read from RTC")
    return _bcd2bin(value)

def _wday_to_onehot(wday: int) -> int:
    # struct_time uses 0=Monday; the register uses bit 0=Sunday
    return 1 << ((wday + 1) % 7)

def _onehot_to_wday(bits: int) -> int:
    for i in range(7):
        if bits & (1 << i):
            return (i + 6) % 7
    raise RuntimeError("Invalid weekday data read from RTC")

class RV8803:
    """Driver for the RV-8803-C7 Real-Time Clock.

    :param ~busio.I2C i2c: The I2C bus the RTC is connected to.
    :param int address: The I2C address of the RTC. Defaults to :const:`0x32`.
    """

    _time = Struct(_SECONDS, "<7B")
    _hundredths = ROUnaryStruct(_HUNDREDTHS, "<B")
    _event_capture = Struct(_HUNDREDTHS_CP, "<2B")

    _alarm_min = RWBits(7, _ALARM_MIN, 0)
    _alarm_min_ae = RWBit(_ALARM_MIN, 7)
    _alarm_hour = RWBits(6, _ALARM_HOUR, 0)
    _alarm_hour_ae = RWBit(_ALARM_HOUR, 7)
    _alarm_weekdays = RWBits(7, _ALARM_DAY, 0)
    _alarm_date = RWBits(6, _ALARM_DAY, 0)
    _alarm_day_ae = RWBit(_ALARM_DAY, 7)

    _timer_value = RWBits(12, _TIMER0, 0, register_width=2)
    _timer_clock = RWBits(2, _EXT, 0)
    _date_alarm_mode = RWBit(_EXT, 6)

    _flags = UnaryStruct(_FLAG, "<B")
    _uf = ROBit(_FLAG, 5)
    _tf = ROBit(_FLAG, 4)
    _af = ROBit(_FLAG, 3)
    _evf = ROBit(_FLAG, 2)
    _v2f = ROBit(_FLAG, 1)

    _stop = RWBit(_CTRL, 0)
    _offset = RWBits(6, _OFFSET, 0, signed=True)

    timer_enabled = RWBit(_EXT, 4)
    """True to run the countdown timer. Switching it from False to True restarts the
    countdown from :attr:`timer_value`. Automatic reloads repeat every
    ``timer_value`` ticks, though the first interval may run one source-clock tick long."""

    update_mode = RWBits(1, _EXT, 5)
    """Periodic update interval: :const:`UPDATE_SECOND` or :const:`UPDATE_MINUTE`."""

    clkout_frequency = RWBits(2, _EXT, 2)
    """CLKOUT frequency: :const:`CLKOUT_32KHZ`, :const:`CLKOUT_1KHZ` or :const:`CLKOUT_1HZ`.
    The CLKOE pin must be tied high for CLKOUT to run."""

    temp_comp_stopped = ROBit(_FLAG, 0)
    """True if low voltage interrupted temperature compensation (V1F). Cleared together with
    :attr:`lost_power`."""

    update_interrupt = RWBit(_CTRL, 5)
    """True to drive the INT pin when a periodic update occurs."""

    timer_interrupt = RWBit(_CTRL, 4)
    """True to drive the INT pin when the countdown timer expires."""

    alarm_interrupt = RWBit(_CTRL, 3)
    """True to drive the INT pin when the alarm matches. Disable it while changing the alarm,
    then clear :attr:`alarm_status` before enabling it again."""

    event_interrupt = RWBit(_CTRL, 2)
    """True to drive the INT pin when an external event is detected."""

    event_capture_enabled = RWBit(_EVCTRL, 7)
    """True to capture the seconds and hundredths in :attr:`event_timestamp` on an external
    event. Keep :attr:`event_reset` False if the captured timestamp must be retained."""

    event_rising_edge = RWBit(_EVCTRL, 6)
    """True to detect external events on a rising edge, False for a falling edge."""

    event_filter = RWBits(2, _EVCTRL, 4)
    """External event filter: :const:`EVENT_FILTER_NONE`, :const:`EVENT_FILTER_4MS`,
    :const:`EVENT_FILTER_16MS` or :const:`EVENT_FILTER_125MS`."""

    event_reset = RWBit(_EVCTRL, 0)
    """True to arm a one-shot reset: the next external event clears the fractional seconds
    and the capture registers, then this returns to False automatically."""

    ram = UnaryStruct(_RAM, "<B")
    """One byte of battery-backed user RAM, 0-255."""

    def __init__(self, i2c: I2C, address: int = _DEFAULT_ADDR) -> None:
        self.i2c_device = I2CDevice(i2c, address)
        _ = self._flags

    def _clear_flags(self, mask: int, value: bool) -> None:
        if value:
            raise ValueError("Status flags can only be cleared by setting them to False")
        # Write 1 to preserve each flag and 0 to clear
        self._flags = _FLAG_MASK & ~mask

    @property
    def datetime(self) -> time.struct_time:
        """The current date and time as a `time.struct_time`. Setting it holds the clock while
        the calendar is written, restarts it and clears :attr:`lost_power`. Years must be in the
        range 2000-2099 and ``tm_wday`` (0=Monday) must be set."""
        regs = self._time
        if _bcd2bin(regs[0]) == 59:
            confirm = self._time
            if _bcd2bin(confirm[0]) != 59:
                regs = confirm
        return time.struct_time(
            (
                2000 + _check_bcd(regs[6], 99),
                _check_bcd(regs[5], 12),
                _check_bcd(regs[4], 31),
                _check_bcd(regs[2], 23),
                _check_bcd(regs[1], 59),
                _check_bcd(regs[0], 59),
                _onehot_to_wday(regs[3]),
                -1,
                -1,
            )
        )

    @datetime.setter
    def datetime(self, value: time.struct_time) -> None:
        if not 2000 <= value.tm_year <= 2099:
            raise ValueError("Year must be in the range 2000-2099")
        if not 0 <= value.tm_wday <= 6:
            raise ValueError("tm_wday must be in the range 0-6")
        regs = (
            _bin2bcd(value.tm_sec),
            _bin2bcd(value.tm_min),
            _bin2bcd(value.tm_hour),
            _wday_to_onehot(value.tm_wday),
            _bin2bcd(value.tm_mday),
            _bin2bcd(value.tm_mon),
            _bin2bcd(value.tm_year - 2000),
        )
        self._stop = True
        try:
            self._time = regs
        finally:
            self._stop = False
        self._clear_flags(_V1F | _V2F, False)

    @property
    def hundredths(self) -> int:
        """Hundredths of the current second, 0-99. Cleared whenever the seconds are written."""
        return _check_bcd(self._hundredths, 99)

    @property
    def lost_power(self) -> bool:
        """True if low voltage made the time unreliable (V2F). Set to False to clear both
        power flags; setting :attr:`datetime` also clears them. A missing coin cell is not
        detected while VIN powers the RTC."""
        return self._v2f

    @lost_power.setter
    def lost_power(self, value: bool) -> None:
        self._clear_flags(_V1F | _V2F, value)

    @property
    def running(self) -> bool:
        """True if timekeeping is running and no power loss has been flagged."""
        return not self._stop and not self._v2f

    @property
    def alarm(self) -> tuple[time.struct_time, int]:
        """The alarm as a ``(time.struct_time, mode)`` tuple, where mode is one of the
        ``ALARM_*`` constants selecting which fields must match.

        Only ``tm_min``, ``tm_hour`` and one day field are used. In date mode (see
        :attr:`alarm_date`) ``tm_mday`` holds the date and ``tm_wday`` is -1. In weekday mode
        (see :attr:`alarm_weekdays`) ``tm_wday`` holds the lowest selected weekday
        (0=Monday, or -1 if none are selected) and ``tm_mday`` is -1.

        Setting it writes the day field that matches the current mode, so a multi-day weekday
        mask is replaced by ``tm_wday``. To match several weekdays, set
        :attr:`alarm_weekdays` after this. Disable :attr:`alarm_interrupt` while changing the
        alarm."""
        minute = _bcd2bin(self._alarm_min)
        hour = _bcd2bin(self._alarm_hour)
        if self._date_alarm_mode:
            mday = _bcd2bin(self._alarm_date)
            wday = -1
        else:
            mday = -1
            weekdays = self._alarm_weekdays
            wday = _onehot_to_wday(weekdays) if weekdays else -1
        mode = self._alarm_min_ae | self._alarm_hour_ae << 1 | self._alarm_day_ae << 2
        return (time.struct_time((2000, 1, mday, hour, minute, 0, wday, -1, -1)), mode)

    @alarm.setter
    def alarm(self, value: tuple[time.struct_time, int]) -> None:
        alarm_time, mode = value
        if not ALARM_MIN_HOUR_DAY <= mode <= ALARM_EVERY_MINUTE:
            raise ValueError("Alarm mode must be one of the ALARM_* constants")
        if not 0 <= alarm_time.tm_min <= 59:
            raise ValueError("tm_min must be in the range 0-59")
        if not 0 <= alarm_time.tm_hour <= 23:
            raise ValueError("tm_hour must be in the range 0-23")
        # AE bits are inverted in hardware: 1 ignores a field, 0 matches it
        self._alarm_min = _bin2bcd(alarm_time.tm_min)
        self._alarm_min_ae = bool(mode & 0x01)
        self._alarm_hour = _bin2bcd(alarm_time.tm_hour)
        self._alarm_hour_ae = bool(mode & 0x02)
        if self._date_alarm_mode:
            if not 1 <= alarm_time.tm_mday <= 31:
                raise ValueError("tm_mday must be in the range 1-31")
            self._alarm_date = _bin2bcd(alarm_time.tm_mday)
        else:
            if not 0 <= alarm_time.tm_wday <= 6:
                raise ValueError("tm_wday must be in the range 0-6")
            self._alarm_weekdays = _wday_to_onehot(alarm_time.tm_wday)
        self._alarm_day_ae = bool(mode & 0x04)

    @property
    def alarm_weekdays(self) -> int | None:
        """Weekdays the alarm matches, as a mask of :const:`SUNDAY` through :const:`SATURDAY`
        or None when the alarm is in date mode. Setting it switches the alarm
        to weekday mode."""
        if self._date_alarm_mode:
            return None
        return self._alarm_weekdays

    @alarm_weekdays.setter
    def alarm_weekdays(self, value: int) -> None:
        if not 0x01 <= value <= 0x7F:
            raise ValueError("Weekday mask must select at least one of SUNDAY-SATURDAY")
        self._date_alarm_mode = False
        self._alarm_weekdays = value

    @property
    def alarm_date(self) -> int | None:
        """Day of the month the alarm matches, 1-31, or None when the alarm is in weekday
        mode."""
        if not self._date_alarm_mode:
            return None
        return _bcd2bin(self._alarm_date)

    @alarm_date.setter
    def alarm_date(self, value: int) -> None:
        if not 1 <= value <= 31:
            raise ValueError("Alarm date must be in the range 1-31")
        self._date_alarm_mode = True
        self._alarm_date = _bin2bcd(value)

    @property
    def alarm_status(self) -> bool:
        """True if the alarm has fired. Set to False to clear it."""
        return self._af

    @alarm_status.setter
    def alarm_status(self, value: bool) -> None:
        self._clear_flags(_AF, value)

    @property
    def timer_value(self) -> int:
        """Countdown timer preset in ticks of :attr:`timer_clock`, 1-4095.
        Setting it stops the timer; set :attr:`timer_enabled` to True to
        start it."""
        return self._timer_value

    @timer_value.setter
    def timer_value(self, value: int) -> None:
        if not 1 <= value <= 4095:
            raise ValueError("Timer value must be in the range 1-4095")
        self.timer_enabled = False
        self._timer_value = value

    @property
    def timer_clock(self) -> int:
        """Countdown timer clock source: :const:`TIMER_4096HZ`, :const:`TIMER_64HZ`,
        :const:`TIMER_1HZ` or :const:`TIMER_1_60HZ`. Setting it stops the timer; set
        :attr:`timer_enabled` to True to start it."""
        return self._timer_clock

    @timer_clock.setter
    def timer_clock(self, value: int) -> None:
        if not TIMER_4096HZ <= value <= TIMER_1_60HZ:
            raise ValueError("Timer clock must be one of the TIMER_* constants")
        # The clock source may only change while the timer is stopped
        self.timer_enabled = False
        self._timer_clock = value

    @property
    def timer_status(self) -> bool:
        """True if the countdown timer has expired. Set to False to clear it."""
        return self._tf

    @timer_status.setter
    def timer_status(self, value: bool) -> None:
        self._clear_flags(_TF, value)

    @property
    def update_status(self) -> bool:
        """True if a periodic update has occurred. Set to False to clear it."""
        return self._uf

    @update_status.setter
    def update_status(self, value: bool) -> None:
        self._clear_flags(_UF, value)

    @property
    def event_timestamp(self) -> tuple[int, int]:
        """The time of the last captured external event as a ``(seconds, hundredths)`` tuple.
        A later event can replace the capture. Disable :attr:`event_capture_enabled`
        before reading if events may recur."""
        hundredths, seconds = self._event_capture
        return (_check_bcd(seconds, 59), _check_bcd(hundredths, 99))

    @property
    def event_status(self) -> bool:
        """True if an external event has been detected. Set to False to clear it."""
        return self._evf

    @event_status.setter
    def event_status(self, value: bool) -> None:
        self._clear_flags(_EVF, value)

    @property
    def calibration(self) -> int:
        """Clock offset calibration, -32 to 31, in steps of 0.2384 ppm. Values outside this
        range are clamped to the nearest end."""
        return self._offset

    @calibration.setter
    def calibration(self, value: int) -> None:
        value = max(-32, min(31, value))
        self._offset = value & 0x3F

    def reset(self) -> None:
        """Restart the current second by pulsing the prescaler reset. The time, date and
        settings are preserved."""
        self._stop = True
        self._stop = False
