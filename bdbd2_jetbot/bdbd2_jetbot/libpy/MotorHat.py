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
# Adapted from https://www.waveshare.net/w/upload/9/9a/Motor_Driver_HAT_Code.7z

import time

from PCA9685 import PCA9685
import smbus

FORWARD = 0
BACKWARD = 1

class MotorDriver():
    """Class to interface with Waveshare Motor hat"""
    def __init__(self, address=0x40, debug=False):
        """Constructor Method"""
        self.PWMA = 0
        self.AIN1 = 1
        self.AIN2 = 2
        self.PWMB = 5
        self.BIN1 = 3
        self.BIN2 = 4
        self.pwm = PCA9685(address, debug=debug)
        self.pwm.setPWMFreq(50)

    def run(self, motor:int, direction:int, speed:int):
        """Run a motor (integer version).
        
        :param int motor: Motor to run (0 or 1)
        :param int direction Go forward (0) or backwards (1)
        :param in speed: how fast (0 - 100)
        """
        if not motor in (0, 1):
            raise ValueError('motor must be 0 or 1')
        if not direction in (0, 1):
            raise ValueError('direction must be 0 or 1')
        if speed < 0 or speed > 100:
            raise ValueError('Speed must be between 0 and 100')

        if(motor == 0):
            self.pwm.setDutycycle(self.PWMA, speed)
            if direction == FORWARD:
                self.pwm.setLevel(self.AIN1, 0)
                self.pwm.setLevel(self.AIN2, 1)
            elif direction == BACKWARD:
                self.pwm.setLevel(self.AIN1, 1)
                self.pwm.setLevel(self.AIN2, 0)
        else:
            self.pwm.setDutycycle(self.PWMB, speed)
            if direction == FORWARD:
                self.pwm.setLevel(self.BIN1, 0)
                self.pwm.setLevel(self.BIN2, 1)
            elif direction == BACKWARD:
                self.pwm.setLevel(self.BIN1, 1)
                self.pwm.setLevel(self.BIN2, 0)

    def runf(self, motor:int, velocity:float):
        """Run a motor (float version).
        
        :param int motor: Motor to run (0 or 1)
        :param float velocity: how fast (between -1.0 and +1.0)
        """


        if not motor in (0, 1):
            raise ValueError('motor must be 0 or 1')
        if velocity < -1.0 or velocity > 1.0:
            raise ValueError('velocity must be between -1.0 and 1.0')
        direction = BACKWARD if velocity < 0.0 else FORWARD
        speed = int(100 * abs(velocity))
        self.run(motor, direction, speed)

    def stop(self):
        """Stop all motors"""
        self.pwm.setDutycycle(self.PWMA, 0)
        self.pwm.setDutycycle(self.PWMB, 0)

    def exit(self):
        self.pwm.exit_PCA9685()

if __name__ == '__main__':
    # i2c address of waveshare motor hat in bdbd power board
    ADDR = 0x44
    FORWARD = 0

    try:
        motorDriver = MotorDriver(address=ADDR, debug=False)
        # control 2 motors
        motorDriver.run(0, FORWARD, 100)
        motorDriver.run(1, FORWARD, 100)
        print("Motors running forward at max speed, use ctl-C to quit")
        time.sleep(3)
        print('all stop')
        motorDriver.stop()
        time.sleep(1.0)
        motorDriver.runf(0, .5)
        motorDriver.runf(1, -.5)
        print('Motor 0 forward half, motor 1 backward half')
        time.sleep(3.0)
        motorDriver.runf(0, .75)
        motorDriver.runf(1, 0.0)
        print('running motor 0 (A) at 75%, motor 1 off')
        time.sleep(3.0)
        motorDriver.runf(1, .75)
        motorDriver.runf(0, 0.0)
        print('running motor 1 (B) at 75%, motor 0 off')
        time.sleep(3.0)

        motorDriver.stop()

    except KeyboardInterrupt:    
        print("\r\nctrl + c:")

    except Exception as e:
        print('\r\n***Error: {}'.format(e))

    finally:
        motorDriver.stop()
        motorDriver.exit()
        exit()
