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
"""Test sayit node messages and services"""

import time
import threading

from bdbd2_msgs.srv import SayText
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

class Tester(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(String, 'sayit', 1)
        self.sub = self.create_subscription(Bool, 'sayit/talking', self.talking_sub_cb, 1)
        self.client = self.create_client(SayText, 'sayit/service')

    def talking_sub_cb(self, msg):
        print(f'Got talking response: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = Tester('test_sayit')
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    spinner = threading.Thread(target=rclpy.spin, args=[node], daemon=True)
    spinner.start()

    # Test using a message
    print('Test using a message')
    msg = String()
    msg.data = "Say it with a message"
    node.pub.publish(msg)
    time.sleep(2.0)

    request = SayText.Request()
    request.text = "say it with a service"
    node.client.call(request)

    rclpy.shutdown()
    spinner.join()

if __name__ == '__main__':
    main()
