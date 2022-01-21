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
"""Exercise the robot drive motors using ros messages"""

import time
import threading

from bdbd2_msgs.msg import MotorsRaw
import rclpy
from rclpy.node import Node

class Tester(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(MotorsRaw, 'motors/cmd_raw', 1)

values = [
    (.25, .25),
    (.5, .5),
    (.75,.75),
    (1.0, 1.0),
    (1.0, 0.0),
    (0.0, -1.0),
    (-1.0, -1.0),
    (-.5, .5),
    (.25, -.25),
    (0.0, 0.0)
]

def main(args=None):
    rclpy.init(args=args)
    node = Tester('test_motors')
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    spinner = threading.Thread(target=rclpy.spin, args=[node], daemon=True)
    spinner.start()

    # Test using a message
    print('Test using a message')
    msg = MotorsRaw()
    for (left, right) in values:
        msg.left = float(left)
        msg.right = float(right)
        node.pub.publish(msg)
        time.sleep(1.0)

    rclpy.shutdown()
    spinner.join()

if __name__ == '__main__':
    main()
