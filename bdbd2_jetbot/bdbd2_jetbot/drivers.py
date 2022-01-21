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
"""
This file contains a class implementing a single ROS2 node for interfacing with robot hardware.

Overview
========
This ROS2 node provides interfaces to hardware on the Jetbot-derived BDBD robot.
Most of these drivers are best available in Python, hence this node is also in Python.

Interfaces to these hardware items:
    - Pan/Tilt hat for the camera

Topic Subscriptions
-------------------

- ``pantilt`` (bdbd2_msgs.msg.PanTilt): A request to set the pan
  and tilt of the camera to specified angles.

Services Provided
-----------------
- ``set_pan_tilt`` (bdbd2_msgs.srv.SetPanTilt): A request to set the pan and tilt of the camera
  to specified angles, with an empty response when done.
- ``get_pan_tilt`` (bdbd2_msgs.srv.GetPanTilt): Reads the current value of camera pan, tilt.
"""

from queue import Queue, Empty
import threading
import time
import traceback

from Adafruit_MotorHAT import Adafruit_MotorHAT
from bdbd2_msgs.msg import MotorsRaw, PanTilt
from bdbd2_msgs.srv import GetPanTilt, SetPanTilt
from bdbd2_jetbot.libpy.PCA9685 import PCA9685
import rclpy
from rclpy.node import Node

# General constants
NODE_NAME = 'bdbd2_drivers'

# Pan/Tilt constants
# See RKJ 2021-03-04 p 78 for earlier angle corrections
# Use bdbd_docker/test/ptcal_apriltag.py for center calibration
# centers
# Raw value for true calibrated center
PANC = 99.3
TILTC = 52.5
# Raw uncalibrated center
PAN_CENTER = 90
TILT_CENTER = 45
# correction to add to raw pan, tilt to get true
PAN_CORR = PAN_CENTER - PANC
TILT_CORR = TILT_CENTER - TILTC
PANTILT_DP = 2 # maximum degrees per interval for pan, half this for tilt
D_TO_R = 3.1415926535 / 180. # degrees to radians
PANTILT_PERIOD = 0.01 # Rate in seconds that the pantilt thread moves the mechanism
SETTLE_SECONDS = 0.10 # allow the pan tilt to settle before sending service reponse

# Motor constants
motor_left_ID = 1
motor_right_ID = 2

class Drivers(Node):
    """Class for a ROS2 node that interfaces with basic hardware on BDBD robot"""

    def __init__(self):
        """Constructor method"""
        super().__init__(NODE_NAME)

        # General setup
        self.loginfo = self.get_logger().info
        self.logwarn = self.get_logger().warn
        self.logerr = self.get_logger().error
        self.loginfo('Starting drivers node')

        # Pan/Tilt setup
        self.pantilt_queue = Queue()
        self.pantilt_sub = self.create_subscription(PanTilt, 'pantilt', self.pantilt_sub_cb, 1)
        self.set_pantilt_service = self.create_service(
            SetPanTilt, 'set_pan_tilt', self.handle_set_pan_tilt
        )
        self.get_pantilt_service = self.create_service(
            GetPanTilt, 'get_pan_tilt', self.handle_get_pan_tilt
        )
        self.pan = PAN_CENTER
        self.tilt = TILT_CENTER

        # Motors setup
        # Adapted from https://github.com/dusty-nv/jetbot_ros/blob/master/scripts/jetbot_motors.py
        # Adapted by R. Kent James <kent@caspia.com> for bdbd robot
        motor_driver = Adafruit_MotorHAT(i2c_bus=1)
        self.motor_left = motor_driver.getMotor(motor_left_ID)
        self.motor_right = motor_driver.getMotor(motor_right_ID)
        # stop the motors as precaution
        self.all_stop()
        self.motors_sub = self.create_subscription(MotorsRaw, 'motors/cmd_raw', self.motors_sub_cb, 1)

    def motors_sub_cb(self, msg:MotorsRaw):
        """Callback for MotorsRaw message"""
        self.loginfo(f'MotorsRaw msg ({msg.left}, {msg.right})')
        self.set_speed(motor_left_ID, msg.left)
        self.set_speed(motor_right_ID, msg.right)

    def set_speed(self, motor_ID:int, value:float):
        """Set the speed of main robot motors
        
        :param motor_ID: which motor, left=1 right=2
        :param value:    motor speed, -1.0 to 1.0
        """
        # bdbd motors are wired backwards, so reverse value
        value = -value
        max_pwm = 255.0 # This allows for full 12 volt output
        speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

        if motor_ID == 1:
            motor = self.motor_left
        elif motor_ID == 2:
            motor = self.motor_right
        else:
            self.logerr('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
            return
        
        MAX_ERRORS = 4
        success = False
        for count in range(MAX_ERRORS):
            count += 1
            try:
                motor.setSpeed(speed)
                if value > 0:
                    motor.run(Adafruit_MotorHAT.FORWARD)
                else:
                    motor.run(Adafruit_MotorHAT.BACKWARD)
                success = True
            except:
                self.logwarn('Motor setSpeed error, retrying')
            if success:
                break
        if not success:
            self.logerr('Motor setSpeed failed')

    def all_stop(self):
        """Stops all main robot driver motors"""
        self.set_speed(motor_left_ID,  0)
        self.set_speed(motor_right_ID,  0)

    def handle_get_pan_tilt(self, request, response):
        """Handler for GetPanTilt service"""
        if request.raw:
            response.pan = self.pan
            response.tilt = self.tilt
        else:
            response.pan = self.pan + PAN_CORR
            response.tilt = self.tilt + TILT_CORR
        return response

    def handle_set_pan_tilt(self, request, response):
        """Handler for SetPanTilt service request"""
        self.loginfo(f'SetPanTilt request {request.pan}, {request.tilt}, {request.raw}')
        responseQueue = Queue()
        self.pantilt_queue.put([request, responseQueue])
        # wait for something to be put in queue
        responseQueue.get()
        # allow the mechanism to settle a little
        time.sleep(SETTLE_SECONDS)
        return response

    def pantilt_sub_cb(self, msg:PanTilt):
        """Callback for PanTilt message"""
        self.loginfo(f'PanTilt msg ({msg.pan}, {msg.tilt}, {msg.raw})')
        self.pantilt_queue.put([msg, None])

    def pantilt_run(self):
        """
        Thread to limit pantilt slew rate

        Read a queue with pantilt movement requests. A false entry in the queue
        message signals a thread exit.
        """

        pca = PCA9685()
        pca.setPWMFreq(50)
        # self.pan, self.tilt are the raw values
        # these are set to slightly off center to force initial motion
        self.pan = PANC - 3
        self.tilt = TILTC - 2
        target_pan = self.pan
        target_tilt = self.tilt
        responseQueue = None
        pca.setRotationAngle(1, self.pan)
        pca.setRotationAngle(0, self.tilt)

        while True:
            panTiltMsg = None
            while panTiltMsg is None:
                if not rclpy.ok():
                    break
                try:
                    panTiltMsg, responseQueue = self.pantilt_queue.get(timeout=1.0)
                except Empty:
                    pass

            # use False to exit queue
            if not panTiltMsg or not rclpy.ok():
                break

            if panTiltMsg.raw:
                target_pan = panTiltMsg.pan
                target_tilt = panTiltMsg.tilt
            else:
                target_pan = panTiltMsg.pan - PAN_CORR
                target_tilt = panTiltMsg.tilt - TILT_CORR

            while self.pan != target_pan or self.tilt != target_tilt:
                start = time.time()
                if self.pan < target_pan:
                    npan = min(target_pan, self.pan + PANTILT_DP)
                else:
                    npan = max(target_pan, self.pan - PANTILT_DP)

                if self.tilt < target_tilt:
                    ntilt = min(target_tilt, self.tilt + PANTILT_DP/2)
                else:
                    ntilt = max(target_tilt, self.tilt - PANTILT_DP/2)

                try:
                    pca.setRotationAngle(1, npan)
                    self.pan = npan
                    pca.setRotationAngle(0, ntilt)
                    self.tilt = ntilt

                except:
                    self.logerr(traceback.format_exc())
                    break

                time.sleep(max(0, start - time.time()))

            if responseQueue:
                responseQueue.put(None)
                responseQueue = None

        # Thread shutdown
        pca.setRotationAngle(1, PAN_CENTER)
        pca.setRotationAngle(0, TILT_CENTER)
        time.sleep(.1) # Let the pca driver do its thing
        pca.exit_PCA9685()
        print('Exiting pantilt thread')

    def run(self):
        """Main program for the node"""
        pantilt_thread = threading.Thread(target=self.pantilt_run)
        pantilt_thread.start()

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

        # signal thread to exit
        self.pantilt_queue.put([False, None])
        self.all_stop()
        time.sleep(.1)
        pantilt_thread.join()
        self.destroy_node()
        print('drivers node shutdown')

if __name__ == '__main__':
    # Testing startup
    rclpy.init()
    drivers = Drivers()
    drivers.run()
    if rclpy.ok():
        rclpy.shutdown()
