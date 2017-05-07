#!/usr/bin/env python3
""" Robot arm lib for controlling customized PhantomX Reactor
"""
from robotlib.ax12.ax12 import Ax12

from robotlib.common import constants


class PhantomX(object):
    """ PhantomX class for controlling customized PhantomX Reactor
    """
    def __init__(self):
        self.ax12 = Ax12()

    def grip(self, location):
        """ Move grip to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 500)
        self.ax12.moveSpeed(constants.MOTOR_ID_GRIP, position, 200)

    def get_grip(self):
        """ Return location of servo
        """
        position = self.ax12.readPosition(constants.MOTOR_ID_GRIP) / 500

        return position

    def set_grip_state(self, state):
        """ Set torque state of servo
        """
        if not isinstance(state, bool):
            raise ValueError("State must be of type bool")

        return self.ax12.setTorqueStatus(constants.MOTOR_ID_GRIP, state)

    def wrist(self, location):
        """ Move wrist to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_WRIST, position, 200)

    def get_wrist(self):
        """ Return location of servo
        """
        position = (
            (self.ax12.readPosition(constants.MOTOR_ID_WRIST) - 200) /
            600
        )

        return position

    def set_wrist_state(self, state):
        """ Set torque state of servo
        """
        if not isinstance(state, bool):
            raise ValueError("State must be of type bool")

        return self.ax12.setTorqueStatus(constants.MOTOR_ID_WRIST, state)

    def hand(self, location):
        """ Move hand to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_HAND, position, 200)

    def get_hand(self):
        """ Return location of servo
        """
        position = (
            (self.ax12.readPosition(constants.MOTOR_ID_HAND) - 200) /
            600
        )

        return position

    def set_hand_state(self, state):
        """ Set torque state of servo
        """
        if not isinstance(state, bool):
            raise ValueError("State must be of type bool")

        return self.ax12.setTorqueStatus(constants.MOTOR_ID_HAND, state)

    def elbow(self, location):
        """ Move elbow to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_ELBOW[0], position, 200)
        position = int((1 - location) * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_ELBOW[1], position, 200)

    def get_elbow(self):
        """ Return location of servo
        """
        position = (
            (self.ax12.readPosition(constants.MOTOR_ID_ELBOW[0]) - 200) /
            600
        )

        return position

    def set_elbow_state(self, state):
        """ Set torque state of servo
        """
        if not isinstance(state, bool):
            raise ValueError("State must be of type bool")

        self.ax12.setTorqueStatus(constants.MOTOR_ID_ELBOW, state)
        self.ax12.setTorqueStatus(constants.MOTOR_ID_ELBOW, state)

        return True

    def shoulder(self, location):
        """ Move shoulder to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_SHOULDER[0], position, 200)
        position = int((1 - location) * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_SHOULDER[1], position, 200)

    def get_shoulder(self):
        """ Return location of servo
        """
        position = (
            (self.ax12.readPosition(constants.MOTOR_ID_SHOULDER[0]) - 200) /
            600
        )

        return position

    def set_shoulder_state(self, state):
        """ Set torque state of servo
        """
        if not isinstance(state, bool):
            raise ValueError("State must be of type bool")

        self.ax12.setTorqueStatus(constants.MOTOR_ID_SHOULDER[0], state)
        self.ax12.setTorqueStatus(constants.MOTOR_ID_SHOULDER[1], state)

        return True

    def pivot(self, location):
        """ Move pivot to desired location
        """
        if location < 0 or location > 1:
            raise ValueError("Incorrect value for location")
        position = int(location * 600 + 200)
        self.ax12.moveSpeed(constants.MOTOR_ID_PIVOT, position, 200)

    def get_pivot(self):
        """ Return location of servo
        """
        position = (
            (self.ax12.readPosition(constants.MOTOR_ID_PIVOT) - 200) /
            600
        )

        return position

    def set_pivot_state(self, state):
        """ Set torque state of servo
        """
        if not isinstance(state, bool):
            raise ValueError("State must be of type bool")

        return self.ax12.setTorqueStatus(constants.MOTOR_ID_PIVOT, state)
