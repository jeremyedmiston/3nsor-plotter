__author__ = 'anton'

import time
from collections import deque
import ev3dev.auto as ev3
import smbus
import socket


def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


class BrickPiPowerSupply(object):

    @staticmethod
    def measured_voltage():
        """
        Reads the digital output code of the MCP3021 chip on the BrickPi+ over i2c.
        Some bit operation magic to get a voltage floating number.

        If this doesnt work try this on the command line: i2cdetect -y 1
        The 1 in there is the bus number, same as in bus = smbus.SMBus(1)
        Google the resulting error.

        :return: voltage (float)
        """

        try:
                bus = smbus.SMBus(1)            # SMBUS 1 because we're using greater than V1.
                address = 0x48
                # time.sleep(0.1) #Is this necessary?

                # read data from i2c bus. the 0 command is mandatory for the protocol but not used in this chip.
                data = bus.read_word_data(address, 0)

                # from this data we need the last 4 bites and the first 6.
                last_4 = data & 0b1111 # using a byte mask
                first_6 = data >> 10 # left shift 10 because data is 16 bits

                # together they make the voltage conversion ratio
                # to make it all easier the last_4 bits are most significant :S
                vdata = ((last_4 << 6) | first_6)

                # Now we can calculate the battery voltage like so:
                voltage = vdata * 0.0179 * 1000000    # This is an empirical number for voltage conversion.

                return voltage

        except:
                return 0.0


def clamp(n, clamp_range):
    """
    Given a number and a range, return the number, or the extreme it is closest to.

    :param n: number
    :return: number
    """
    (minn, maxn) = clamp_range
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


class PIDControl(object):
    """
    Helper class that remembers the integral and derivative of an error and uses that to calculate
    feedback power.
    """

    def __init__(self, Kp=1.0, Ti=0.0, Td=0.0, Kp_neg_factor=1, max_out=100, max_integral=100, direction=1, precision=15):
        self.direction = direction
        self.__Kp = Kp
        self.Kp_neg_factor = Kp_neg_factor
        self.Kp_neg = Kp * Kp_neg_factor    # Different feedback factor in the negative direction.
        self.Ti = Ti
        self.Td = Td
        self.zero = 0
        self.__current = 0
        self.precision = precision
        self.set_point = 0         # This also initializes other properties using setter
        self.max_out = max_out
        self.max_i = max_integral

    @property
    def Kp(self):
        return self.__Kp

    @Kp.setter
    def Kp(self, Kp):
        self.__Kp = Kp
        self.Kp_neg = Kp * self.Kp_neg_factor

    @property   # getter
    def error(self):
        return self.__set_point - self.current

    @property   # getter
    def current(self):
        return self.__current - self.zero

    @current.setter   # setter
    def current(self, point):
        self.__current = point

    @property   # getter
    def set_point(self):
        return self.__set_point

    @set_point.setter
    def set_point(self, target):
        # Setter, python style!
        # Not only set a new target, but also reset other steering factors
        self.__set_point = target * self.direction     # Change direction if necessary
        self.integral = 0                           # Reset integral part
        self.prev_error = self.error                # Reset errors
        self.timestamp = time.time()-0.02           # Reset derivative timer
        self.start_time = time.time()               # Set starttime for ramping up
        self.history = deque(maxlen=3)
        self.intervals = deque(maxlen=3)

    @property
    def target_reached(self):
        return abs(self.error) < self.precision

    @property
    def speed(self):
        return sum(self.history)/sum(self.intervals)

    def calc_power(self):
        """
        Saves a timestamp, integral and previous error and does PID calculations.
        Always feed this to a motor.
        :return: int motor power
        """

        #get error & save timestamps
        error = self.error
        self.history.append(error)

        # calculate integral
        dt = time.time() - self.timestamp
        self.intervals.append(dt)
        self.integral += error * dt
        self.integral = clamp(self.integral,(-self.max_i,self.max_i)) #when driving a long time, this number can get too high.

        #calculate derivative.
        self.derivative = (error - self.prev_error) / dt

        #save error & time for next time.
        self.prev_error = error
        self.timestamp = time.time()

        # Use different proportional factor for running backwards if the load is different.
        if error < 0:
            Kp = self.Kp_neg
        else:
            Kp = self.Kp

        output = Kp * ( error + self.integral * self.Ti + self.Td * self.derivative )
        if output < -2:
            output -= 8
        elif output > 2:
            output += 8

        self.output = output
        return int(clamp(output,(-self.max_out,self.max_out)))


class PIDMotor(ev3.Motor):
    def __init__(self, port=None, name='*', Kp=3.0, Ki=0.0, Kd=0.0, brake=0, verbose=False, speed_reg=False, **kwargs):
        ev3.Motor.__init__(self, port, name)
        self.positionPID = PIDControl(Kp=Kp, Ti=Ki, Td=Kd, max_out=100)
        self.brake = brake
        self.verbose = verbose
        self.speed_reg = speed_reg
        self.power = 0

    @property
    def position_sp(self):
        return self.positionPID.set_point

    @position_sp.setter
    def position_sp(self,tgt):
        self.positionPID.set_point = tgt

    def stop(self):
        self.power = 0
        super().stop()

    def run(self):
        self.positionPID.current = self.position
        pospower = self.positionPID.calc_power()
        if self.speed_reg:
            self.run_forever(speed_sp = pospower)
        else:
            self.duty_cycle_sp = pospower
            self.run_direct()

    def run_at_speed_sp(self, spd):
        self.power += (spd-self.speed)*0.05
        self.duty_cycle_sp = int(clamp((self.power), (-100, 100)))
        self.run_direct()

    def run_for_time(self, time_in_s, speed):
        end_time = time.time() + time_in_s
        while time.time() < end_time:
            self.run_at_speed_sp(speed)

    def run_to_abs_pos(self, position_sp=None):
        if position_sp is not None:
            self.positionPID.set_point = position_sp
        while not self.positionPID.target_reached:
            self.run()

        t_end = time.time() + self.brake
        while time.time() < t_end:
            #print "Braking"
            self.run()

        self.stop()