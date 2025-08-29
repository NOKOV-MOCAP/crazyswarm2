import rclpy

from . import genericJoystick
from .crazyflie import CrazyflieServer, TimeHelper


class Crazyswarm:

    def __init__(self):
        rclpy.init()

        self.allcfs = CrazyflieServer()
        self.timeHelper = TimeHelper(self.allcfs)

        self.input = genericJoystick.Joystick(self.timeHelper)
        self._active = True

    def shutdown(self):  
        if self._active and rclpy.ok():
            rclpy.shutdown()
        self._active = False
