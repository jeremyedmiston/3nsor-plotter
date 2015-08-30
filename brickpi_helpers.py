__author__ = 'anton'

import time

#### Math helpers ####

class plot_installation(object):
    def __init__(self,l_rope_0,r_rope_0,attachment_distance, pulley_diam = 4.4):
        self.l_rope_0 = l_rope_0
        self.r_rope_0 = r_rope_0
        self.att_dist = attachment_distance

        #self.pulley = pulley_diam
        # now some math to calculate the rest of the plotter parameters
        self.cm_to_deg = 180 / 3.1415 * 2 / pulley_diam * 24 / 8  #angle-in-deg = l-in-cm/(diameter/2) * 360 /(2*PI) * num_teeth_large_gear / num_teeth_small_gear
        self.v_margin = self.triangle_area(l_rope_0, r_rope_0, attachment_distance) / attachment_distance * 2  #height of triangle
        self.h_margin = (l_rope_0 ** 2 - self.v_margin ** 2) ** 0.5  #pythagoras to find distance from triangle point to left doorframe
        self.canvas_size = attachment_distance - 2 * self.h_margin

    def motor_targets_from_norm_coords(self,x_norm, y_norm):
        x,y = self.normalized_to_global_coords(x_norm,y_norm)
        return self.motor_targets_from_coords(x,y)

    def motor_targets_from_coords(self,x, y):
        l_rope = (x ** 2 + y ** 2) ** 0.5
        r_rope = ((self.att_dist - x) ** 2 + y ** 2) ** 0.5
        l_target = (l_rope - self.l_rope_0) * self.cm_to_deg
        r_target = (r_rope - self.r_rope_0) * self.cm_to_deg

        return int(l_target), int(r_target)

    def coords_from_motor_pos(self,l_motor,r_motor):
        l_rope = l_motor / self.cm_to_deg + self.l_rope_0
        r_rope = r_motor / self.cm_to_deg + self.r_rope_0
        y = self.triangle_area(l_rope,r_rope,self.att_dist)*2/self.att_dist
        x = (l_rope**2 - y**2)**0.5
        y_norm = (y - self.v_margin)/self.canvas_size
        x_norm = (x - self.h_margin)/self.canvas_size

        return x_norm,y_norm

    def normalized_to_global_coords(self,x_norm,y_norm):
        # convert normalized coordinates to global coordinates
        x = x_norm * self.canvas_size + self.h_margin
        y = y_norm * self.canvas_size + self.v_margin

        return x,y

    def triangle_area(self,a, b, c):
        """
        Calculate the area of a triangle by the lengths of it's sides using Heron's formula

        :param a: Length of side a
        :param b: Length of side b
        :param c: Length of side c
        :return: area (float)
        """
        half_p = (a + b + c) / 2
        return (half_p * (half_p - a) * (half_p - b) * (half_p - c)) ** 0.5

class Throttler(object):
    """
    Helper class to make sure a certain amount of time has passed before entering the next pass trough a loop
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