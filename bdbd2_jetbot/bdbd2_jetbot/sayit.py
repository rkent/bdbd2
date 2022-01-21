#!/usr/bin/env python
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
"""Class implementing ROS2 node to generate audio speech from text

Topic Subscriptions
-------------------
- ``sayit`` (std_msgs/String): Say the text

Topics Published
----------------
- ``sayit/talking`` (std_msgs/Bool): True published when sound starts, False when finished.

Services Provided
-----------------
- ``sayit/service`` (bdbd2_msgs.srv.SayText): Say the text, with an empty response when done.

"""

import threading
from queue import Queue, Empty

from bdbd2_jetbot.libpy.googleTTS import GoogleTTS
from bdbd2_msgs.srv import SayText
from espeakng import ESpeakNG
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

# General Constants
#ENGINE = 'espeak-ng'
ENGINE = 'google'
ESPEAK_VOLUME = 40
NODE_NAME = 'sayit'

PERIOD = 0.1 # update rate in seconds

class Sayit(Node):
    """Class for a ROS2 node that generates speech audio from text"""

    def __init__(self):
        """Constructor method"""
        super().__init__(NODE_NAME)

        # General setup
        self.loginfo = self.get_logger().info
        self.logwarn = self.get_logger().warn
        self.logerr = self.get_logger().error
        self.loginfo(f'Starting {NODE_NAME} node')
        self.queue = Queue()

        # Speech methods setup
        self.espeak = ESpeakNG(voice='en-gb-x-gbclan')
        self.googleTTS = GoogleTTS()
        self.espeak.volume = ESPEAK_VOLUME

        # ROS2 interfaces
        self.sayit_sub = self.create_subscription(String, 'sayit', self.sayit_sub_cb, 10)
        self.talking_pub = self.create_publisher(Bool, 'sayit/talking', 1)
        self.sayit_service = self.create_service(SayText, 'sayit/service', self.handle_sayit)

        # Initial message with status
        self.pub_talking(False)

    def sayit_sub_cb(self, msg):
        """Callback for sayit message"""
        text = msg.data
        self.loginfo('Topic request to say: ' + text)
        self.queue.put([text, None])

    def pub_talking(self, isTalking):
        """Publish the talking message"""
        msg = Bool()
        msg.data = bool(isTalking)
        self.talking_pub.publish(msg)

    def handle_sayit(self, request, response):
        """Manage sayit/service"""
        responseQueue = Queue()
        text = request.text
        self.loginfo('Service request to say: ' + text)
        self.queue.put([text, responseQueue])
        responseQueue.get()
        # an empty response, signifying we are through talking
        return(response)

    def process_queue(self):
        """
        Thread to process text requests from a queue

        Reads reqests from a queue, saying the text. A None entry in the queue text
        signals a thread exit.
        """
        while rclpy.ok():
            text, responseQueue = self.queue.get()
            if text is None:
                break
            self.pub_talking(True)
            if ENGINE == 'google':
                self.googleTTS.say(text)
            else:
                self.espeak.say(text, sync=True)
            if responseQueue:
                responseQueue.put(None)
            self.pub_talking(False)
        
    def run(self):
        """Main program for the node"""
        sayit_thread = threading.Thread(target=self.process_queue, daemon=True)
        sayit_thread.start()

        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass

        # signal thread to exit
        self.queue.put([None, None])
        sayit_thread.join()
        self.destroy_node()
        print(f'{NODE_NAME} shutdown')

if __name__ == '__main__':
    # testing startup
    rclpy.init()
    sayit = Sayit()
    sayit.run()
    if rclpy.ok():
        rclpy.shutdown()
