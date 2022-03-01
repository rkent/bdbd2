#!/usr/bin/python3
#
#   Copyright 2022 R. Kent James <kent@caspia.com>
#
#   Licensed under the Apache License, Version 2.0 (the "License");
#   you may not use this file except in compliance with the License.
#   You may obtain a copy of the License at
#
#       http://www.apache.org/licenses/LICENSE-2.0
#
#   Unless required by applicable law or agreed to in writing, software
#   distributed under the License is distributed on an "AS IS" BASIS,
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#   See the License for the specific language governing permissions and
#   limitations under the License.
#
# Inspired by:
# https://github.com/adafruit/Adafruit_CircuitPython_INA260/blob/main/adafruit_ina260.py
# and https://www.waveshare.net/w/upload/9/9a/Motor_Driver_HAT_Code.7z
# which has an MIT license.
#
# Rewritten from the Adafruit-provided coded, as that code required --privileged Docker. It seemed
# unfortunate to require that for a less important function such as this.
#

"""Module to interface with Adafruit INA260 voltage/current breakout board"""
import smbus

def const(value):
    """Do-nothing function for Circuit Python code compatibility"""
    return value

_REG_CONFIG = const(0x00)  # CONFIGURATION REGISTER (R/W)
_REG_CURRENT = const(0x01)  # SHUNT VOLTAGE REGISTER (R)
_REG_BUSVOLTAGE = const(0x02)  # BUS VOLTAGE REGISTER (R)
_REG_POWER = const(0x03)  # POWER REGISTER (R)
_REG_MASK_ENABLE = const(0x06)  # MASK ENABLE REGISTER (R/W)
_REG_ALERT_LIMIT = const(0x07)  # ALERT LIMIT REGISTER (R/W)
_REG_MFG_UID = const(0xFE)  # MANUFACTURER UNIQUE ID REGISTER (R)
_REG_DIE_UID = const(0xFF)  # DIE UNIQUE ID REGISTER (R)


class AveragingCount:
    """Options for ``averaging_count``

    +-------------------------------+------------------------------------+
    | ``AveragingCount``            | Number of measurements to average  |
    +===============================+====================================+
    | ``AveragingCount.COUNT_1``    | 1 (Default)                        |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_4``    | 4                                  |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_16``   | 16                                 |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_64``   | 64                                 |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_128``  | 128                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_256``  | 256                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_512``  | 512                                |
    +-------------------------------+------------------------------------+
    | ``AveragingCount.COUNT_1024`` | 1024                               |
    +-------------------------------+------------------------------------+

    """

    COUNT_1 = const(0x0)
    COUNT_4 = const(0x1)
    COUNT_16 = const(0x2)
    COUNT_64 = const(0x3)
    COUNT_128 = const(0x4)
    COUNT_256 = const(0x5)
    COUNT_512 = const(0x6)
    COUNT_1024 = const(0x7)

    @staticmethod
    def get_averaging_count(avg_count):
        """Retrieve the number of measurements giving value read from register"""
        conv_dict = {0: 1, 1: 4, 2: 16, 3: 64, 4: 128, 5: 256, 6: 512, 7: 1024}
        return conv_dict[avg_count]

def lhswap(value):
    """Swap high and low bytes"""
    value = value & 0xffff
    return (value >> 8) + ((value & 0xff) << 8)

class INA260():

    def __init__(self,
        address:int,
        debug:bool=False,
        average=AveragingCount.COUNT_64
    ):
        """Initializer
        
        :param int address: I2C bus address
        :param bool debug: True to print debug messages (default False)
        :param int average: How many 1.1 ms intervals to average (Use AverageCount class)
        """
        self.bus = smbus.SMBus(1)
        self.address = address
        self.debug = debug

        # Confirm that device is found
        TEXAS_INSTRUMENT_ID = const(0x5449)
        INA260_ID = const(0x2270)
        manufacturer_id = self.bus.read_word_data(self.address, _REG_MFG_UID)
        manufacturer_id = self.read_word(_REG_MFG_UID)
        device_id = self.read_word(_REG_DIE_UID) & 0xfff0
        if debug:
            print('manufacturer_id: 0x{:x}'.format(manufacturer_id))
            print('device_id: 0x{:x}'.format(device_id))

        if manufacturer_id != TEXAS_INSTRUMENT_ID or device_id != INA260_ID:
            raise RuntimeError('INA260 device not found')

        # Configure the averaging counts. We use the default value for the
        # conversion time (1.1ms) so each count is approximately the number
        # of milliseconds averaged.
        config = self.read_word(_REG_CONFIG)
        if (debug):
            print('Initial configuration: 0x{:x}'.format(config))
        AVG_mask = 0x0e00
        not_AVG_mask = 0xffff & ~AVG_mask
        AVG_shift = 9
        config = (config & not_AVG_mask) | (average << AVG_shift)
        if debug:
            print('Desired config 0x{:x}'.format(config))
        self.write_word(_REG_CONFIG, config)
        config = self.read_word(_REG_CONFIG)
        if (debug):
            print('Final configuration: 0x{:x}'.format(config))


    def write(self, reg, value):
        "Writes an 8-bit value to the specified register/address"
        self.bus.write_byte_data(self.address, reg, value)
        if (self.debug):
            print("I2C: Write 0x%02X to register 0x%02X" % (value, reg))

    def read(self, reg):
        "Read an unsigned byte from the I2C device"
        result = self.bus.read_byte_data(self.address, reg)
        if (self.debug):
            print("I2C: Device 0x%X returned 0x%X from reg 0x%X" % (self.address, result & 0xFF, reg))
        return result

    def read_word(self, reg):
        raw = self.bus.read_word_data(self.address, reg)
        return lhswap(raw)

    def write_word(self, reg, value):
        self.bus.write_word_data(self.address, reg, lhswap(value))


    @property
    def voltage(self):
        raw_voltage = self.read_word(_REG_BUSVOLTAGE)
        voltage = raw_voltage * 0.00125
        if self.debug:
            print('raw_voltage: 0x{:x} voltage: {}'.format(raw_voltage, voltage))
        return voltage

    @property
    def current(self):
        raw_current = self.read_word(_REG_CURRENT)
        current = raw_current * 1.25
        if self.debug:
            print('raw_current: 0x{:x} current (ma): {}'.format(raw_current, current))
        return current

    @property
    def power(self):
        raw_power = self.read_word(_REG_POWER)
        power = raw_power * 10.0
        if self.debug:
            print('raw_power: 0x{:x} power (ma): {}'.format(raw_power, power))
        return power

if __name__ == "__main__":
    import time

    # Basic usage and test. With these settings, values update about
    # once every 0.14 seconds
    ina260 = INA260(address=0x41, debug=False)
    start = time.time()
    # Default average time is 1.1ms * 128 = 140 ms. Show changes
    current = 0.0
    for count in range(0, 20):
        while ina260.current == current:
            time.sleep(0.001)
        current = ina260.current
        print('\nTime change: {}'.format(time.time() - start))
        start = time.time()
        print('Voltage: {} volts'.format(ina260.voltage))
        print('Current: {} millamps'.format(ina260.current))
        print('Power: {} milliwatts'.format(ina260.power))

    
