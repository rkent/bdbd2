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
"""Test hearity and sayit node messages and services by echoing shat is heard"""

import time
import threading

from bdbd2_msgs.msg import AngledText
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class Tester(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(String, 'sayit', 1)
        self.sub = self.create_subscription(Bool, 'sayit/talking', self.talking_sub_cb, 1)
        self.hearit_sub = self.create_subscription(AngledText, 'hearit/angled_text', self.hearit_sub_cb, 1)

    def talking_sub_cb(self, msg):
        print(f'Got talking response: {msg}')

    def hearit_sub_cb(self, msg):
        print(f'Got hearit message {msg.text} at angle {msg.direction}')
        self.pub.publish(String(data=msg.text))

def main(args=None):
    rclpy.init(args=args)
    node = Tester('echo_hearsay')
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    spinner = threading.Thread(target=rclpy.spin, args=[node], daemon=True)
    spinner.start()

    print('echoing for 60 seconds')
    start = time.time()
    while rclpy.ok():
        time.sleep(.1)
        if time.time() - start > 60:
            break
    rclpy.shutdown()
    spinner.join()

if __name__ == '__main__':
    main()
