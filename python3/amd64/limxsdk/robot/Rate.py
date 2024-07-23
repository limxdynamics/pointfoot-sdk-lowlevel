"""
@file Rate.py

© [2023] LimX Dynamics Technology Co., Ltd. All rights reserved.
"""

import limxsdk.robot as robot

class Rate(object):
    def __init__(self, frequency: float):
        """
        Constructor for the Rate class.

        Args:
            frequency (float): The frequency in Hz (cycles per second).
        """
        self.native = robot.RateNative(frequency)

    def sleep(self):
        """
        Sleeps for any leftover time in a cycle. Calculated from the last time sleep, reset, or the constructor was called.
        True if the desired rate was met for the cycle, false otherwise.
        """
        return self.native.sleep()

    def reset(self):
        """
        Reset the start time.
        """
        self.native.reset()
