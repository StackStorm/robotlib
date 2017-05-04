#!/usr/bin/env python3
""" Robot arm lib for controlling customized PhantomX Reactor
"""
from robotlib.ax12.ax12 import Ax12

from robotlib.common import constants


class PhantomX(object):
    """ PhantomX class for controlling customized PhantomX Reactor
    """
    def __init__(self):
        self._arm_state = {
            "grip": 0,
            "hand": 0,
            "wrist": 0,
            "elbow": 0,
            "sholder": 0,
        }

        self.ax12 = Ax12()

    def grip(self, location):
        """ Move grip to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 500)
        self.ax12.move(constants.MOTOR_ID_GRIP, position)

    def get_grip(self):
        """ Return location of servo
        """
        return self.ax12.readPosition(constants.MOTOR_ID_GRIP)

    def wrist(self, location):
        """ Move wrist to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_WRIST, position)

    def get_wrist(self):
        """ Return location of servo
        """
        return self.ax12.readPosition(constants.MOTOR_ID_WRIST)

    def hand(self, location):
        """ Move hand to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_HAND, position)

    def get_hand(self):
        """ Return location of servo
        """
        return self.ax12.readPosition(constants.MOTOR_ID_HAND)

    def elbow(self, location):
        """ Move elbow to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_ELBOW[0], position)
        position = int((1 - location) * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_ELBOW[1], position)

    def get_elbow(self):
        """ Return location of servo
        """
        return self.ax12.readPosition(constants.MOTOR_ID_ELBOW[0])

    def sholder(self, location):
        """ Move sholder to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_SHOLDER[0], position)
        position = int((1 - location) * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_SHOLDER[1], position)

    def get_sholder(self):
        """ Return location of servo
        """
        return self.ax12.readPosition(constants.MOTOR_ID_SHOLDER[0])

    def pivot(self, location):
        """ Move pivot to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.move(constants.MOTOR_ID_PIVOT, position)

    def get_pivot(self):
        """ Return location of servo
        """
        return self.ax12.readPosition(constants.MOTOR_ID_PIVOT)

