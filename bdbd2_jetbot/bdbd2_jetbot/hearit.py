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
"""ROS2 node to listen for speech and publish text from the speech"""

import rclpy
from rclpy.node import Node

try:
    from Queue import Queue
except:
    from queue import Queue

import os
import traceback
import speech_recognition as sr
from bdbd2_jetbot.libpy.rerecognizer import ReRecognizer, indexFromName
import json
import threading
from bdbd2_msgs.msg import AngledText
from std_msgs.msg import Bool
from audio_common_msgs.msg import AudioData
import time

# General Constants
RATE = 0.01
NODE_NAME = 'hearit'

class Hearit(Node):
    """Class for a ROS2 node that generates text from speech audio"""

    def __init__(self):
        """Constructor method"""
        super().__init__(NODE_NAME)

        # General setup
        self.loginfo = self.get_logger().info
        self.logwarn = self.get_logger().warn
        self.logerr = self.get_logger().error
        self.loginfo(f'Starting {NODE_NAME} node')
        self.continueThread = True
        self.is_talking = False
        self.mic = None
        self.r = None

    def talking_cb(self, msg):
        self.is_talking = msg.data

    def talking_get(self):
        return self.is_talking

    def run(self):
        self.loginfo('Running node hearit')

        self.textpub = self.create_publisher(AngledText, 'hearit/angled_text', 1)
        self.audiopub = self.create_publisher(AudioData, 'audio', 10)
        self.mikestatuspub = self.create_publisher(Bool, 'mike/status', 1)
        self.talking_sub = self.create_subscription(Bool, 'sayit/talking', self.talking_cb, 10)

        device = indexFromName('respeaker')
        self.mic = sr.Microphone(device_index=device)
        self.r = ReRecognizer(status_cb=self.status_cb, talking_get=self.talking_get)
        self.recognizer = self.r.recognize_google_cloud
        self.google_key = ''
        with open('/secrets/stalwart-bliss-270019-7159f52eb443.json', 'r') as f:
            self.google_key = f.read()
        assert self.google_key, 'Google API read failed'

        self.voiceQueue = Queue()
        voiceThread = threading.Thread(target=self.getVoice)
        voiceThread.daemon = True
        voiceThread.start()

        self.loginfo('hearit entering main loop')
        while voiceThread.is_alive() and rclpy.ok():
            try:
                if not self.voiceQueue.empty():
                    statement, angle = self.voiceQueue.get()
                    self.textpub.publish(AngledText(text=statement, direction=angle))
                rclpy.spin_once(self, timeout_sec=RATE)
            except KeyboardInterrupt:
                break
            except:
                self.logerr(traceback.format_exc())
                break

        self.continueThread = False

    def status_cb(self, status):
        if status:
            self.loginfo('Collecting voice stream')
            self.mikestatuspub.publish(Bool(data=True))
        else:
            self.loginfo('Waiting for voice')
            self.mikestatuspub.publish(Bool(data=False))

    def getVoice(self):
        while self.continueThread:
            angle = -1
            try:
                with self.mic as source:
                    # r.adjust_for_ambient_noise(source)
                    audio, angle = self.r.listen(source)
                    #print(audio.get_wav_data()[0:40])
                    self.loginfo('Sound heard at angle: ' + str(angle))
                    #print('Sound heard at angle: ' + str(angle))
                    if self.audiopub.get_subscription_count() > 0:
                        self.audiopub.publish(AudioData(data=audio.get_wav_data()))
                    text = self.recognizer(audio, credentials_json=self.google_key)
                    self.loginfo('we heard statement: ' + text)
                    self.voiceQueue.put([text, angle])
            except sr.UnknownValueError:
                self.loginfo('sound not recognized as speech')
                self.voiceQueue.put(['', angle])
            except:
                self.logwarn('Error in getVoice: {}'.format(traceback.format_exc()))

if __name__ == '__main__':
    # testing startup
    rclpy.init()
    hearit = Hearit()
    hearit.run()
    if rclpy.ok():
        hearit.loginfo('Node hearit shutting down')
        rclpy.shutdown()
