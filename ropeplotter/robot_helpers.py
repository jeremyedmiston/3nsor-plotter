__author__ = 'anton'

import time
from collections import deque
import ev3dev


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

    def __init__(self, logname="log",to_file=False):
        self.logname = logname
        self.loglist = []
        self.new_line_ready = False
        self.to_file = to_file
        if to_file:
            self.logfile = open("logs/"+self.logname+".csv",'w')

    def log(self, *args):
        self.new_line_ready = False
        self.loglist += list(args)
        #print args

    def newline(self):
        if len(self.loglist) > 0:
            self.lastline = [time.time()] + list(self.loglist)
            self.loglist = []
            self.new_line_ready = True
            if self.to_file:
                self.logfile.write(",".join([str(i) for i in self.lastline])+"\n")

    def log_line(self, *args):
        self.log(*args)
        self.newline()


class MotorPidControl(object):
    """
    Helper class that remembers the integral and derivative of an error and uses that to calculate
    motor power for a servo.
    """

    def __init__(self, motor_port, Kp=2, Ti=0, Td=0, Kp_neg_factor=1, maxpower=255, direction=1, precision=12):
        self.port = motor_port
        self.direction = direction
        self.__Kp = Kp
        self.Kp_neg_factor = Kp_neg_factor
        self.Kp_neg = Kp * Kp_neg_factor    # Different feedback factor in the negative direction.
        self.Ti = Ti
        self.Td = Td
        self.zero = 0
        self.encoder = 0
        self.precision = precision
        self.target = 0         # This also initializes other properties using setter

        self.maxpower = maxpower
        logname = "-".join([str(i) for i in ["motor",motor_port]])
        self.log = Logger(logname, to_file=False)
        self.log.log_line('target','error','output','integral','derivative')

    @property
    def Kp(self):
        return self.__Kp

    @Kp.setter
    def Kp(self, Kp):
        self.__Kp = Kp
        self.Kp_neg = Kp * self.Kp_neg_factor

    @property   # getter
    def error(self):
        return self.__target - self.position

    @property   # getter
    def position(self):
        return self.encoder - self.zero

    @property   # getter
    def target(self):
        return self.__target

    @target.setter  # setter, python style!
    def target(self, target):
        self.__target = target * self.direction
        self.integral = 0
        self.prev_error = self.error
        #self.errors = deque([self.error], maxlen=6)
        self.timestamp = time.time()-0.02
        #self.timestamps = deque([self.timestamp],maxlen=6)

    @property
    def target_reached(self):
        return abs(self.error) < self.precision

    def calc_power(self):
        """
        Saves a timestamp, integral and previous error and does PID calculations.
        Always feed this to a motor.
        :return: int motor power
        """

        #get error & save timestamps
        error = self.error

        # calculate integral
        dt = time.time() - self.timestamp
        self.integral += error * dt
        self.integral = clamp(self.integral,(-self.maxpower/2,self.maxpower/2)) #when driving a long time, this number can get too high.

        #calculate derivative. Use the error value from 6 cycles back, because of jitter.
        derivative = (error - self.prev_error) / dt
        #dt6 = (time.time() - self.timestamps[0])
        #derivative = (error - self.errors[0]) / dt6

        #calculate output
        if error < 0:
            Kp = self.Kp_neg
        else:
            Kp = self.Kp
        output = Kp * ( error + self.integral * self.Ti + self.Td * derivative )   #Ti should be 1/Ti.
        self.log.log_line(self.target, error, output, self.integral, derivative)

        #save error & time for next time.
        self.prev_error = error
        self.timestamp = time.time()
        #self.errors.append(error)
        #self.timestamps.append(time.time())

        return int(clamp(output,(-self.maxpower,self.maxpower)))


class MotorPid(ev3dev.motor):
    def __init__(self, motor_port, Kp=2, Ti=0, Td=0, Kp_neg_factor=1, maxpower=100, direction=1, precision=12):
        self.direction = direction
        self.__Kp = Kp
        self.Kp_neg_factor = Kp_neg_factor
        self.Kp_neg = Kp * Kp_neg_factor    # Different feedback factor in the negative direction.
        self.Ti = Ti
        self.Td = Td
        self.precision = precision
        self.target = 0         # This also initializes other properties using setter
        self.maxpower = maxpower

        logname = "-".join([str(i) for i in ["motor",motor_port]])
        self.log = Logger(logname, to_file=True)
        self.log.log_line('target', 'error', 'output', 'integral', 'derivative', 'reached')
        ev3dev.motor.__init__(self)

    @property
    def Kp(self):
        return self.__Kp

    @Kp.setter
    def Kp(self, Kp):
        self.__Kp = Kp
        self.Kp_neg = Kp * self.Kp_neg_factor

    @property   # getter
    def error(self):
        return self.__target - self.position

    @property   # getter
    def target(self):
        return self.__target

    @target.setter  # setter, python style!
    def target(self, target):
        self.__target = target * self.direction
        self.integral = 0
        self.prev_error = self.error
        self.timestamp = time.time()-0.02

    @property
    def target_reached(self):
        return abs(self.error) < self.precision

    def calc_power(self):
        """
        Saves a timestamp, integral and previous error and does PID calculations.
        Always feed this to a motor.
        :return: int motor power
        """

        #get error & save timestamps
        error = self.error

        # calculate integral
        dt = time.time() - self.timestamp
        self.integral += error * dt
        self.integral = clamp(self.integral,(-self.maxpower/2,self.maxpower/2)) #when driving a long time, this number can get too high.

        #calculate derivative.
        derivative = (error - self.prev_error) / dt

        #calculate output
        if error < 0:
            Kp = self.Kp_neg
        else:
            Kp = self.Kp
        output = Kp * ( error + self.integral * self.Ti + self.Td * derivative )
        self.log.log_line(self.target, error, output, self.integral, derivative, self.target_reached)

        #save error & time for next time.
        self.prev_error = error
        self.timestamp = time.time()

        return int(clamp(output,(-self.maxpower,self.maxpower)))

    def run(self, **kwargs):
        if kwargs['target']:
            self.target = kwargs['target']
        self.run_forever(duty_cycle_sp=self.calc_power())