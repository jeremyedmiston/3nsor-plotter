__author__ = 'anton'

import time
from collections import deque


def clamp(n, (minn, maxn)):
    """
    Given a number and a range, return the number, or the extreme it is closest to.

    :param n: number
    :return: number
    """
    return max(min(maxn, n), minn)


def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.

    val: float or int
    src: tuple
    dst: tuple

    example: print scale(99, (0.0, 99.0), (-1.0, +1.0))
    """
    return (float(val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]


class Throttler(object):
    """
    Helper class to make sure a certain amount of time has passed before entering the next pass trough a loop.
    Allows me to set a 'framerate' and makes sure a loop doesn't run faster than that.
    """

    def __init__(self, framerate):
        self.fps = framerate
        self.timestamp = time.time()

    def throttle(self):
        wait_time = 1.0 / self.fps - (
            time.time() - self.timestamp)  # has enough time passed? If not, this is the remainder
        if wait_time > 0:
            time.sleep(wait_time)
        self.timestamp = time.time()


class Logger(object):
    """
    Helper class that logs events to the console and later maybe to a file.

    log() writes whatever you throw at it to the current line in the logbook

    newline() tells the log it's time to start a new group of events. This makes it easier to reconstruct what happened
    in multiple threads.

    """

    def __init__(self, logname=""):
        self.logname = logname
        self.loglist = []
        self.new_line_ready = False

    def log(self, *args):
        if type(args) == list:
            self.loglist += args
        else:
            self.loglist += [args]
        print args

    def newline(self):
        if len(self.loglist) > 0:
            self.lastline = [self.logname, time.time()] + [self.loglist]
            self.loglist = []
            self.new_line_ready = True
            #TODO Write this to a file

    def get_lastline(self):
        self.new_line_ready = False
        return self.lastline

    def has_new_line(self):
        return self.new_line_ready


class motorPID_control(object):
    """
    Helper class that remembers the integral and derivative of an error and uses that to calculate
    motor power for a servo.
    """

    def __init__(self, motor_port, KP=3, Ti=0, Td=0, maxpower=255):
        self.port = motor_port
        self.Kp = KP
        self.Ti = Ti     #convert ms to seconds
        self.Td = Td     #convert ms to seconds
        self.integral = 0
        self.prev_error = 0
        self.timestamp = time.time()
        self.zero = 0
        self.target = 0
        self.encoder = 0
        self.errors = deque(maxlen=6)
        self.maxpower = maxpower

    @property   # getter
    def error(self):
        return self.target - self.position

    @property   # getter
    def position(self):
        return self.encoder - self.zero

    @property   # getter
    def target(self):
        return self.__target

    @target.setter  # setter, python style!
    def target(self, target):
        self.__target = target
        self.integral = 0
        self.prev_error = 0

    def calc_power(self):
        """
        Saves a timestamp, integral and previous error and does PID calculations.
        Always feed this to a motor.
        :return: int motor power
        """
        error = self.error
        self.errors.append(error)
        dt = time.time() - self.timestamp
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.Kp * ( error + self.Ti * self.integral + self.Td * derivative )   #Ti should be 1/Ti.
        #print error, output,  "integral:", self.integral, "deriv:", derivative
        self.prev_error = error
        self.timestamp = time.time()
        return int(clamp(output,(-self.maxpower,self.maxpower)))