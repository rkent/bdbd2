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

"""Module contains a class to track battery status"""

import threading
import time

from sensor_msgs.msg import BatteryState
# Allowing running locally
try:
    from .INA260 import INA260, AveragingCount
except:
    import os
    import sys
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    sys.path.append(os.path.dirname(SCRIPT_DIR))
    from INA260 import INA260, AveragingCount

SECONDS_PER_HOUR = 3600.

class BatteryStatus():
    "Class to track battery status"

    def __init__(self, address=0x40, filter_seconds=1.0, sample_time=0.05):
        self.filter_seconds = filter_seconds
        self.ina260 = INA260(address=address, average=AveragingCount.COUNT_64)
        self._power = self.ina260.power
        self._milliwatt_hours = 0.0 # Accumulated power usage
        self._voltage = self.ina260.voltage
        self._current = self.ina260.current
        self.active = True
        self.sample_time = sample_time
        self.battery_thread = threading.Thread(target=self.battery_run)
        self.battery_lock = threading.Lock()

    def run(self):
        """Start battery thread"""
        self.battery_thread.start()

    def stop(self):
        """Exit battery thread"""
        self.active = False
        self.battery_thread.join()
        print('Exiting battery_status thread')

    def filter(self, old_value, new_value, elapsed_time):
        """Return a filtered value accounting for new data"""
        factor = max(0, elapsed_time / self.filter_seconds)
        return (1. - factor) * old_value + factor * new_value

    def battery_run(self):
        """Thread runner to maintain average battery state"""
        start = time.time()
        while self.active:
            time.sleep(self.sample_time)
            # instantaneous values
            power = self.ina260.power
            voltage = self.ina260.voltage
            current = self.ina260.current
            now = time.time()
            elapsed_time = now - start
            start = now

            # averaged values
            self.battery_lock.acquire()
            self._power = self.filter(self._power, power, elapsed_time)
            self._voltage = self.filter(self._voltage, voltage, elapsed_time)
            self._current = self.filter(self._current, current, elapsed_time)
            self._milliwatt_hours += power * elapsed_time / SECONDS_PER_HOUR
            self.battery_lock.release()

    @property
    def power(self):
        self.battery_lock.acquire()
        value = self._power
        self.battery_lock.release()
        return value

    @property
    def voltage(self):
        self.battery_lock.acquire()
        value = self._voltage
        self.battery_lock.release()
        return value

    @property
    def current(self):
        self.battery_lock.acquire()
        value = self._current
        self.battery_lock.release()
        return value

    @property
    def milliwatt_hours(self):
        self.battery_lock.acquire()
        value = self._milliwatt_hours
        self.battery_lock.release()
        return value

    def batteryState(self):
        """Generate a current BatteryState message"""
        state = BatteryState()
        self.battery_lock.acquire()
        state.voltage = self._voltage
        state.current = self._current
        state.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        state.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        state.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        state.present = True
        self.battery_lock.release()
        return state

if __name__ == '__main__':
    """Demo and test"""
    from pprint import pprint
    batteryStatus = BatteryStatus(
        address = 0x41,
        filter_seconds=1.0,
        sample_time = 0.05
    )
    try:
        batteryStatus.run()
        for count in range(100):
            print(' ')
            state = batteryStatus.batteryState()
            for attr in (
                'current',
                'voltage',
            ):
                print(f'{attr}: {getattr(state,  attr)}')

            for attr in ('power', 'milliwatt_hours'):
                print(f'{attr}: {getattr(batteryStatus,  attr)}')
            time.sleep(0.2)
    finally:
        batteryStatus.stop()