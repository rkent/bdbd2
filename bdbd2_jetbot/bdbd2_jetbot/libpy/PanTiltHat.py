#!/usr/bin/python3
"""Raspi PCA9685 16-Channel PWM Servo Driver

Software provided with Waveshare Pan/Tilt hat.
Available at https://github.com/waveshare/Pan-Tilt-HAT.
No License specified, but the code seems to be derivative of Adafruit_PWM_Servo_Driver.py
from Pypi Adafruit_MotorHAT which has MIT license specified.

Modified by R. Kent James <kent@caspia.com> All modifications done under Apache 2 license.
"""

from PCA9685 import PCA9685

class PanTiltDriver():
    """Driver for Waveshare pan tilt hat"""

    def __init__(self, address=0x40, debug=False):
        self.pwm = PCA9685(address, debug=debug)
        self.pwm.setPWMFreq(50)

    def setRotationAngle(self, channel, Angle):
      """
          Set the rotation angle of pan or tilt

          :param int channel: 0 for tilt, 1 for pan
          :param int Angle:   pan or tilt angle, 0-90 for tilt, 0-180 for pan

      """
      if(Angle >= 0 and Angle <= 180):
          temp = Angle * (2000 / 180) + 501
          self.pwm.setServoPulse(channel, temp)
      else:
          print("Angle out of range")

    def exit(self):
      self.pwm.exit_PCA9685()

if __name__ == '__main__':
    ADDRESS = 0x40
    # A simple demo of the pan/tilt
    PAN_MIN = 10
    PAN_MAX = 170
    PAN_CENTER = 90
    TILT_MIN = 5
    TILT_MAX = 85
    TILT_CENTER = 45
    import time
    print('simple test of pan/tilt')
    panTiltDriver = PanTiltDriver(address=ADDRESS, debug=True)
    panTiltDriver.setRotationAngle(1, 0)
    try:
        for i in range(PAN_MIN, PAN_MAX, 2): 
            panTiltDriver.setRotationAngle(1, i)
            j = TILT_MIN + (2 * i % (TILT_MAX - TILT_MIN))
            panTiltDriver.setRotationAngle(0, j)   
            time.sleep(0.1)
    except Exception as e:
        print('Exception: {}'.format(e))
    finally:
        panTiltDriver.setRotationAngle(1, PAN_CENTER)
        panTiltDriver.setRotationAngle(0, TILT_CENTER)
        time.sleep(0.5)
        panTiltDriver.exit()
        print('PCA9685 test end')
