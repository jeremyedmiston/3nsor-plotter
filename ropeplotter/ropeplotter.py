__author__ = 'anton'

import time

from PIL import Image

from robot_helpers import MotorPidControl, clamp
import ev3dev


class RopePlotter(object):
    def __init__(self,l_rope_0, r_rope_0, attachment_distance, pulley_diam=4.4, Kp=2.2, Ti=0.2, Td=0.02, Kp_neg=1.3, maxpower=200, RPi_mode=True):
        self.__l_rope_0 = l_rope_0
        self.__r_rope_0 = r_rope_0
        self.__att_dist = attachment_distance
        self.pulley_diam = pulley_diam
        self.direction = 1 # -1 is for reversing motors
        self.calc_constants()
        self.maxpower = maxpower
        self.RPi_mode = RPi_mode

        if RPi_mode:
        # Start the BrickPi
            self.pen_motor = ev3dev.motor(ev3dev.OUTPUT_D)
            self.left_motor = ev3dev.motor(ev3dev.OUTPUT_B)
            self.right_motor = ev3dev.motor(ev3dev.OUTPUT_C)
        else:
            self.pen_motor = ev3dev.motor('outA')
            self.left_motor = ev3dev.motor('outB')
            self.right_motor = ev3dev.motor('outC')
            # BrickPi can't do this, alas:
            self.pen_motor.set(speed_regulation_enabled='on', stop_command='brake')
            self.left_motor.set(speed_regulation_enabled='on', stop_command='brake')
            self.right_motor.set(speed_regulation_enabled='on', stop_command='brake')

        #TODO Check if motors are connected and raise error if not.
        self.drive_motors = [self.left_motor, self.right_motor]

        # Initialise motor control
        #TODO Refactor this, as most of the stuff is not needed anymore on ev3dev.
        left_motor_control = MotorPidControl('outB', Kp, Ti, Td, Kp_neg=Kp_neg, maxpower=maxpower, direction=self.direction)
        right_motor_control = MotorPidControl('outC', Kp, Ti, Td, Kp_neg=Kp_neg, maxpower=maxpower, direction=self.direction)
        self.pen_motor_control = MotorPidControl('outA')
        self.drive_motor_controls = [left_motor_control, right_motor_control]

        # Set starting point
        self.set_motor_zeroes()
        self.precision = 9  # Motors stop running when they are within +/-9 degrees of target.

    # Getters & setters for plotter properties. Python style, Baby!
    # After setting these, some calculations need to be done, that's why I define special functions
    # And decorate them as setters and getters.
    # TODO Refactor for ev3dev.
    @property
    def Kp(self):
        return 0

    @Kp.setter
    def Kp(self,Kp):
        for motor in self.drive_motor_controls:
            motor.Kp = Kp

    @property
    def Ti(self):
        return 0

    @Ti.setter
    def Ti(self,Ki):
        for motor in self.drive_motor_controls:
            motor.Ti = Ki

    @property
    def Td(self):
        return 0

    @Td.setter
    def Td(self,Kd):
        for motor in self.drive_motor_controls:
            motor.Td = Kd

    @property
    def l_rope_0(self):
        return self.__l_rope_0

    @l_rope_0.setter
    def l_rope_0(self,length):
        self.__l_rope_0 = length
        self.calc_constants()

    @property
    def r_rope_0(self):
        return self.__r_rope_0

    @r_rope_0.setter
    def r_rope_0(self,length):
        self.__r_rope_0 = length
        self.calc_constants()

    @property
    def att_dist(self):
        return self.__att_dist

    @att_dist.setter
    def att_dist(self,length):
        self.__att_dist = length
        self.calc_constants()

    def calc_constants(self):
        # now some math to calculate the rest of the plotter parameters
        #angle-in-deg = l-in-cm/(diameter/2) * 360 /(2*PI) * num_teeth_large_gear / num_teeth_small_gear
        #-2 is BrickPi weirdness. Encoders run backwards in half degrees.
        self.cm_to_deg = -2 * 180 / 3.1415 * 2 / self.pulley_diam * 24 / 8
        self.v_margin = self.triangle_area(self.__l_rope_0, self.__r_rope_0, self.__att_dist) / self.__att_dist * 2  #height of triangle
        self.h_margin = (self.__l_rope_0 ** 2 - self.v_margin ** 2) ** 0.5  #pythagoras to find distance from triangle point to left doorframe
        self.canvas_size = self.__att_dist - 2 * self.h_margin

    @staticmethod
    def triangle_area(a, b, c):
        """
        Calculate the area of a triangle by the lengths of it's sides using Heron's formula

        :param a: Length of side a
        :param b: Length of side b
        :param c: Length of side c
        :return: area (float)
        """
        half_p = (a + b + c) / 2
        return (half_p * (half_p - a) * (half_p - b) * (half_p - c)) ** 0.5

    # Calculations for global to local coordinates and back.
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

    # Movement functions
    def pen_up(self):
        self.pen_motor.run_timed(time_sp=150, duty_cycle_sp=30)

    def pen_down(self):
        self.pen_motor.run_timed(time_sp=150, duty_cycle_sp=-30)

    def set_motor_zeroes(self):
        for motor in self.drive_motors + [self.pen_motor]:
            motor.position = 0

    def move_to_coord(self,x,y):
        motor_b_target, motor_c_target  = self.motor_targets_from_coords(x, y)
        #print "Moving to ", x, ",", y, "(At", motor_b_target, motor_c_target, ")"
        self.move_to_targets((motor_b_target, motor_c_target))


    def move_to_norm_coord(self, x_norm, y_norm):
        motor_b_target, motor_c_target = self.motor_targets_from_norm_coords(x_norm, y_norm)
        #print "Moving to ", x_norm, ",", y_norm, "(At", motor_b_target, motor_c_target, ")"
        self.move_to_targets((motor_b_target, motor_c_target))

    def move_to_targets(self, targets):
        for i in range(2):
            self.drive_motors[i].run_to_abs_pos(position_sp=targets[i])
        #Now wait for the motors to reach their targets
        while 1:
            if self.close_enough(self.drive_motors[0], targets[0]) and self.close_enough(self.drive_motors[1], targets[1]): break
            time.sleep(0.09)

    def close_enough(self, test, target):
        return target - self.precision < test < target + self.precision


    # Advanced plotting functions by chaining movement functions
    def test_drive(self):
        # A little disturbance in the force
        self.move_to_norm_coord(0.05,0.05)
        self.move_to_norm_coord(0.0,0.05)
        self.move_to_norm_coord(0.0,0.0)

    def plot_from_file(self, filename):
        """
        Generator function for plotting from coords.csv file. After each next() it returns the pct done of the plotting
        This way the plotting can easily be aborted and status can be given. Gotta love python for this.
        Usage:

        gen = plotter.plot_from_file(myfile)
        while 1:
            try:
                pct_done = next(gen)
            except StopIteration:
                break

        :param filename: str
        :return: percentage done: float
        """
        coords = open(filename)
        num_coords = int(coords.readline())  #coords contains it's length on the first line.

        #drive to the first coordinate
        self.pen_up()
        # read from file
        x_norm, y_norm = [float(n) for n in coords.readline().split(",")]
        #move
        self.move_to_norm_coord(x_norm, y_norm)
        self.pen_down()
        for i in range(num_coords - 1):
            # read from file
            x_norm, y_norm = [float(n) for n in coords.readline().split(",")]
            #move
            self.move_to_norm_coord(x_norm, y_norm)
            yield float(i+1)/num_coords*100

        coords.close()
        self.pen_up()
        self.move_to_norm_coord(0, 0)
        yield 100

    def plot_circles(self, num_circles=12):
        UP = 0
        DOWN = -30

        im = Image.open("uploads/picture.jpg").convert("L")
        w, h = im.size
        pixels = im.load()

        r_min = (self.h_margin**2+self.v_margin**2)**0.5
        r_max = ((self.h_margin+self.canvas_size)**2 + (self.v_margin+self.canvas_size)**2)**0.5
        r_step = (r_max-r_min)/num_circles

        for right_side_mode in [0, 1]: # Multiply all terms that only apply to the right side circles with right_side_mode
            if right_side_mode:
                drive_motor, anchor_motor = self.drive_motors
            else:
                anchor_motor, drive_motor = self.drive_motors

            # First draw circles with left anchor point as center.
            for i in range(1, num_circles, 2):
                # Move to the starting point at x,y
                # Caculate where a circle with radius r_min+r_step*i crosses the left margin.
                x = self.h_margin + (right_side_mode * self.canvas_size)
                y = ((r_min+r_step*i)**2 - self.h_margin ** 2) ** 0.5   # This is the same left and right
                if y > self.v_margin+self.canvas_size:
                    # We reached the bottom, now we check where circles cross the bottom margin
                    if right_side_mode:
                        x = self.h_margin + self.canvas_size - ((r_min + r_step*i) ** 2 - (self.v_margin + self.canvas_size) ** 2) ** 0.5
                    else:
                        x = ((r_min + r_step*i) ** 2 - (self.v_margin + self.canvas_size) ** 2) ** 0.5
                    y = self.v_margin+self.canvas_size  # This is the same left and right
                self.move_to_coord(x,y)

                #turn on right motor, slowly, to draw circles upwards
                drive_motor.run_forever(duty_cycle_sp=60)

                # Now calculate coordinates continuously until we reach the top, or right side of the canvas
                # Motor B is off, so let's get it's encoder only once
                while 1:
                    # Look at the pixel we're at and move pen up or down accordingly
                    x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position, self.drive_motors[1].position)
                    pixel_location = tuple([clamp(c,(0,1)) * w for c in (x_norm, y_norm)])
                    print "looking at pixel ", pixel_location
                    if pixels[pixel_location] < 80: # About 33% gray
                        self.pen_motor.run_to_abs_position(position_sp=DOWN)
                    else:
                        self.pen_motor.run_to_abs_position(position_sp=UP)

                    if y_norm <= 0:
                        break # reached the top
                    if (not right_side_mode) and x_norm >= 1:
                        break # reached the right side
                    if right_side_mode and x_norm <= 0:
                        break

                # Pen up
                self.pen_motor.run_to_abs_position(position_sp=UP)

                # Yield to allow pause/stop and show percentage
                yield i*(50.0+right_side_mode*50.0)/num_circles

                #Good, now move to the next point and roll down.
                if right_side_mode:
                    x = self.h_margin + self.canvas_size - ((r_min + r_step*(i+1)) ** 2 - self.v_margin ** 2) ** 0.5
                else:
                    x = ((r_min + r_step*(i+1)) ** 2 - self.v_margin ** 2) ** 0.5
                y = self.v_margin

                if (not right_side_mode) and x > (self.h_margin + self.canvas_size): # Reached right side
                    x = self.h_margin + self.canvas_size
                    y = ((r_min+r_step*(i+1)) ** 2 - (self.h_margin+self.canvas_size) ** 2) ** 0.5
                if right_side_mode and x < self.h_margin: # Reached left side
                    x = self.h_margin
                    y = ((r_min+r_step*(i+1)) ** 2 - (self.h_margin+self.canvas_size) ** 2) ** 0.5
                self.move_to_coord(x, y)

                #turn on right motor, slowly to draw circles from right to left.
                drive_motor.run_forever(duty_cycle_sp=-40)

                # Calculate coordinates continuously until we reach the top, or right side of the canvas
                while 1:
                    # Look at the pixel we're at and move pen up or down accordingly
                    x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position, self.drive_motors[1].position)
                    pixel_location = tuple([clamp(c,(0,1)) * w for c in (x_norm, y_norm)])
                    if pixels[pixel_location] < 80: # About 33% gray
                        self.pen_motor.run_to_abs_position(position_sp=DOWN)
                    else:
                        self.pen_motor.run_to_abs_position(position_sp=UP)

                    if y_norm >= 1:
                        break # reached the bottom
                    if x_norm <= 0 and not right_side_mode:
                        break # reached the left side
                    if x_norm >= 1 and right_side_mode:
                        break
                    time.sleep(0.02)

                # Pen up
                self.pen_motor.run_to_abs_position(position_sp=UP)

                # Yield to allow pause/stop and show percentage
                yield (i+1)*(50.0+right_side_mode*50.0)/num_circles



    # Calibration & manual movement functions
    def left_fwd(self):
        self.left_motor.run_forever(duty_cycle_sp=70 * self.direction)

    def left_stop(self):
        self.left_motor.stop()

    def left_back(self):
        self.left_motor.run_forever(duty_cycle_sp=-40 * self.direction)

    def right_fwd(self):
        self.right_motor.run_forever(duty_cycle_sp=70 * self.direction)

    def right_stop(self):
        self.right_motor.stop()

    def right_back(self):
        self.right_motor.run_forever(duty_cycle_sp=-40 * self.direction)

    def stop_all_motors(self):
        for motor in self.drive_motors + [self.pen_motor]:
            motor.stop()

    def get_voltage(self):
        if self.RPi_mode:
            #TODO smbus stuff
            return 0.0
        else:
            return ev3dev.power_supply.battery.measured_voltage / 1000000.0