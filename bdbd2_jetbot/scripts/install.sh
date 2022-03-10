#!/bin/bash
# Installs prerequisites that are not available using rosdep
# See https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml
# and https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml
# for available rosdep packages.

sudo apt install -y espeak-ng python3-pyaudio
sudo pip3 install google-cloud-texttospeech
sudo pip3 install google-cloud-speech
sudo pip3 install py-espeak-ng

# Other config issues:
# 1) google credentials in /secrets
