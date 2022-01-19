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
(This is module-level documentation. This docstring must be inserted
before any code in the module.)

Overview
========
This ROS2 node provides interfaces to hardware on the Jetbot-derived BDBD robot.

Most of these drivers are best available in Python, hence this node is also in Python.
"""

from queue import Queue, Empty
import threading
import time
import traceback

from bdbd2_msgs.msg import PanTilt
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
PANTILT_RATE = 100 # Rate in hertz that the pantilt thread moves the mechanism
SETTLE_SECONDS = 0.10 # allow the pan tilt to settle before sending service reponse


class Drivers(Node):
    """Class for a ROS2 node that interfaces with basic hardware on BDBD robot"""

    def __init__(self):
        """Constructor method"""
        super().__init__(NODE_NAME)

        # General setup
        self.loginfo = self.get_logger().info
        self.logerr = self.get_logger().error
        self.loginfo('Starting drivers node')

        # Pan/Tilt setup

        self.pantilt_queue = Queue()
        self.pantilt_sub = self.create_subscription(PanTilt, 'pantilt', self.pantilt_sub_cb, 10)
    
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
        pantilt_rate = self.create_rate(PANTILT_RATE)

        pca = PCA9685()
        pca.setPWMFreq(50)
        # self.pan, self.tilt are the raw values
        # these are set to slightly off center to force initial motion
        pan = PANC - 3
        tilt = TILTC - 2
        target_pan = pan
        target_tilt = tilt
        pca.setRotationAngle(1, pan)
        pca.setRotationAngle(0, tilt)

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

            while pan != target_pan or tilt != target_tilt:
                if pan < target_pan:
                    npan = min(target_pan, pan + PANTILT_DP)
                else:
                    npan = max(target_pan, pan - PANTILT_DP)

                if tilt < target_tilt:
                    ntilt = min(target_tilt, tilt + PANTILT_DP/2)
                else:
                    ntilt = max(target_tilt, tilt - PANTILT_DP/2)

                try:
                    pca.setRotationAngle(1, npan)
                    pan = npan
                    pca.setRotationAngle(0, ntilt)
                    tilt = ntilt

                except:
                    self.logerr(traceback.format_exc())
                    break

                pantilt_rate.sleep()

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
