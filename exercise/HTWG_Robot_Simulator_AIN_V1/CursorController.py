# class CursorController.
#
# O. Bittel
# V 2.0; 9.3.2016

from math import *


class CursorController:
    def __init__(self, win):
        self.win = win
        self.d_v = 0.1
        self.d_omega = 2.0 * (pi/180.0)
        self.v = 0
        self.omega = 0
        self.v_Max = 1.0
        self.omega_max = 90.0 * (pi/180.0)

    def setDefaultSpeed(self, v, omega):
        self.v = v
        self.omega = omega

    def getSpeed(self):
        key = self.win.checkKey()
        if key == "":
            pass
        elif key == "Up":
            if self.v < self.v_Max:
                self.v += self.d_v
        elif key == "Down":
            if self.v > -self.v_Max:
                self.v -= self.d_v
        elif key == "Left":
            if self.omega < self.omega_max:
                self.omega += self.d_omega
        elif key == "Right":
            if self.omega > -self.omega_max:
                self.omega -= self.d_omega
        elif key == "Return":
            self.omega = 0
        elif key == "space":
            self.v = 0
            self.omega = 0
        elif key == "Escape":
            return None
        return [self.v, self.omega]
