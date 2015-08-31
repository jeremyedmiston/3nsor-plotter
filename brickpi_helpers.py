__author__ = 'anton'

import time

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

    def __init__(self, motor_port, KP=1, KI=0.0, KD=0.0):
        self.port = motor_port
        self.Kp = KP
        self.Ki = KI
        self.Kd = KD
        self.integral = 0
        self.prev_error = 0
        self.timestamp = time.time()
        self.zero = 0
        self.target = 0
        self.encoder = 0

    @property
    def error(self):
        return self.target - self.position

    @property
    def position(self):
        return self.encoder - self.zero

    @property
    def target(self):
        return self.__target

    @target.setter
    def target(self, target):
        self.__target = target
        self.integral = 0
        self.prev_error = 0

    def get_power(self):
        error = self.error
        dt = time.time() - self.timestamp
        self.integral = error * dt
        derivative = (error - self.prev_error) / dt
        #print self.port, error, self.integral, derivative
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        self.timestamp = time.time()
        return int(output)