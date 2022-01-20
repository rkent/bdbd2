#!/usr/bin/env python3
"""Exercise the pantilt mechanism using ros messages"""

import time
import threading

from bdbd2_msgs.msg import PanTilt
from bdbd2_msgs.srv import SetPanTilt
import rclpy
from rclpy.node import Node

class Tester(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(PanTilt, 'pantilt', 10)

values = [
    (40, 50),
    (170, 10),
    (170, 80),
    (10, 80),
    (10, 10),
    (170, 10),
    (10, 80),
    (170, 80),
    (10, 10),
    (90, 45)
]

def main(args=None):
    rclpy.init(args=args)
    node = Tester('test_pantilt')
    # https://answers.ros.org/question/358343/rate-and-sleep-function-in-rclpy-library-for-ros2/
    spinner = threading.Thread(target=rclpy.spin, args=[node], daemon=True)
    spinner.start()

    # Test using a service
    print('Test using a service')
    client = node.create_client(SetPanTilt, 'set_pan_tilt')
    for (pan, tilt) in values:
        request = SetPanTilt.Request()
        request.pan = float(pan)
        request.tilt = float(tilt)
        client.call(request)

    # Test using a message
    print('Test using a message')
    msg = PanTilt()
    for (pan, tilt) in values:
        msg.pan = float(pan)
        msg.tilt = float(tilt)
        msg.raw = True
        node.pub.publish(msg)
        time.sleep(1.0)

    rclpy.shutdown()
    spinner.join()

if __name__ == '__main__':
    main()
