#!/usr/bin/env python3
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
"""Simple test of motors using Adafruit drivers and no ROS"""

DURATION = 5.0
LEFT = -0.8
RIGHT = .8

# Adapted from https://github.com/dusty-nv/jetbot_ros/blob/master/scripts/jetbot_motors.py
# Adapted by R. Kent James <kent@caspia.com> for bdbd robot
from Adafruit_MotorHAT import Adafruit_MotorHAT

# setup motor controller
motor_driver = Adafruit_MotorHAT(i2c_bus=1)

motor_left_ID = 1
motor_right_ID = 2

motor_left = motor_driver.getMotor(motor_left_ID)
motor_right = motor_driver.getMotor(motor_right_ID)

# sets motor speed between [-1.0, 1.0]
def set_speed(motor_ID, value):
    #max_pwm = 115.0
    max_pwm = 255.0 # 12 volts
    speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

    if motor_ID == motor_left_ID:
        motor = motor_left
    else:
        motor = motor_right
    
    motor.setSpeed(speed)

    if value > 0:
        motor.run(Adafruit_MotorHAT.FORWARD)
    else:
        motor.run(Adafruit_MotorHAT.BACKWARD)

# stops all motors
def all_stop():
    motor_left.setSpeed(0)
    motor_right.setSpeed(0)

    motor_left.run(Adafruit_MotorHAT.RELEASE)
    motor_right.run(Adafruit_MotorHAT.RELEASE)

if __name__ == '__main__':
    import time
    try:
        left = LEFT
        right = RIGHT
        print('motor on')
        # negative because of BDBD wiring
        set_speed(motor_left_ID,  -left)
        set_speed(motor_right_ID, -right)
        time.sleep(DURATION)
    except:
        pass
    all_stop()
