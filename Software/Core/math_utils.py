import numpy as np

class DegMath:
    @staticmethod
    def sin(deg): return np.sin(np.radians(deg))
    @staticmethod
    def cos(deg): return np.cos(np.radians(deg))
    @staticmethod
    def tan(deg): return np.tan(np.radians(deg))
    @staticmethod
    def arcsin(x): return np.degrees(np.arcsin(x))
    @staticmethod
    def arccos(x): return np.degrees(np.arccos(x))
    @staticmethod
    def arctan(tan): return np.degrees(np.arctan(tan))
    @staticmethod
    def arctan2(y, x): return np.degrees(np.arctan2(y, x))

# Easing functions
def ease_in_out(t): return 3 * t**2 - 2 * t**3
def ease_out_quad(t): return 1 - (1 - t)**2
def ease_in_quad(t): return t**2
def ease_sin(t): return 0.5 * (1 - np.cos(np.pi * t))
