__author__ = 'anton'

from brickpi_helpers import motorPID_control
from BrickPi import *


class rope_plotter(object):
    def __init__(self,l_rope_0,r_rope_0,attachment_distance, pulley_diam=4.4, Kp=1.5, Ti=0.8, Td=0.05, maxpower=200):
        self.l_rope_0 = l_rope_0
        self.r_rope_0 = r_rope_0
        self.att_dist = attachment_distance

        #self.pulley = pulley_diam
        # now some math to calculate the rest of the plotter parameters
        #angle-in-deg = l-in-cm/(diameter/2) * 360 /(2*PI) * num_teeth_large_gear / num_teeth_small_gear
        #-2 is BrickPi weirdness. Encoders run backwards in half degrees.
        self.cm_to_deg = -2 * 180 / 3.1415 * 2 / pulley_diam * 24 / 8
        self.v_margin = self.triangle_area(l_rope_0, r_rope_0, attachment_distance) / attachment_distance * 2  #height of triangle
        self.h_margin = (l_rope_0 ** 2 - self.v_margin ** 2) ** 0.5  #pythagoras to find distance from triangle point to left doorframe
        self.canvas_size = attachment_distance - 2 * self.h_margin

        #Start the BrickPi
        BrickPiSetup()  # setup the serial port for communication
        BrickPi.MotorEnable[PORT_A] = 1  #Enable the Motor A
        BrickPi.MotorEnable[PORT_B] = 1  #Enable the Motor B
        BrickPi.MotorEnable[PORT_C] = 1  #Enable the Motor C
        BrickPi.MotorEnable[PORT_D] = 1  #Enable the Motor D

        no_values = 1
        while no_values:
            # Make sure we have something before we start running
            # So we wait until no_values goes 0, which means values updated OK
            no_values = BrickPiUpdateValues()


        left_motor = motorPID_control(PORT_B, Kp, Ti, Td, maxpower=maxpower)
        right_motor = motorPID_control(PORT_C, Kp, Ti, Td, maxpower=maxpower)
        self.drive_motors = [left_motor, right_motor]
        self.set_motor_zero()
        self.precision = 5

    @property
    def Kp(self):
        return 0

    @Kp.setter
    def Kp(self,Kp):
        for motor in self.drive_motors:
            motor.Kp = Kp

    @property
    def Ti(self):
        return 0

    @Ti.setter
    def Ti(self,Ki):
        for motor in self.drive_motors:
            motor.Ti = Ki

    @property
    def Td(self):
        return 0

    @Td.setter
    def Td(self,Kd):
        for motor in self.drive_motors:
            motor.Td = Kd





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


    def pen_up(self):
        self.move_motor_for_time(PORT_D,30)


    def pen_down(self):
        self.move_motor_for_time(PORT_D,-30)


    def set_motor_zero(self):
        for motor in self.drive_motors:
            motor.encoder = int(BrickPi.Encoder[motor.port])
            motor.zero = int(BrickPi.Encoder[motor.port])
            print "Encoder zero position set to", motor.zero, "For motor at port:", motor.port


    def move_to_coord(self,x,y):
        motor_b_target, motor_c_target  = self.motor_targets_from_coords(x, y)
        print "Moving to ", x, ",", y, "(At", motor_b_target, motor_c_target, ")"
        self.move_to_targets((motor_b_target, motor_c_target))


    def move_to_norm_coord(self, x_norm, y_norm):
        motor_b_target, motor_c_target = self.motor_targets_from_norm_coords(x_norm, y_norm)
        print "Moving to ", x_norm, ",", y_norm, "(At", motor_b_target, motor_c_target, ")"
        self.move_to_targets((motor_b_target, motor_c_target))


    def move_to_targets(self, targets):
        # set targets
        for i in range(2):
            self.drive_motors[i].target = targets[i]

        # run motors until targets are reached
        while 1:
            BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
            for motor in self.drive_motors:
                #get motor positions
                motor.encoder = BrickPi.Encoder[motor.port]

                #set motor speed accordingly
                speed = motor.calc_power()
                BrickPi.MotorSpeed[motor.port] = speed

            if (abs(self.drive_motors[0].error) < self.precision) and (abs(self.drive_motors[1].error) < self.precision):
                #we got where we want to be. Time for the next coordinate.
                for motor in self.drive_motors:
                    BrickPi.MotorSpeed[motor.port] = 0
                BrickPiUpdateValues()
                break

            #We're done calculating and setting all motor speeds!
            time.sleep(0.02)



    def plot_from_file(self, filename):
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

        coords.close()
        self.pen_up()
        self.move_to_norm_coord(0, 0)


    def plot_circles(self, num_circles=12):
        r_min = (self.h_margin**2+self.v_margin**2)**0.5
        r_max = ((self.h_margin+self.canvas_size)**2 + (self.v_margin+self.canvas_size)**2)**0.5
        r_step = (r_max-r_min)/num_circles

        motor_B, motor_C = self.drive_motors #unpack

        for i in range(1, num_circles, 2):
            x = self.h_margin
            y = ((r_min+r_step*i)**2 - self.h_margin ** 2) ** 0.5
            print "y:",y, "vm:",self.v_margin,"rmin:",r_min,"step:",r_step
            if y > self.v_margin+self.canvas_size:
                x = ((r_min + r_step*i) ** 2 - (self.v_margin + self.canvas_size) ** 2) ** 0.5
                y = self.v_margin+self.canvas_size
                print "Phase 3: Rollin' up!"
            else:
                print "Phase 1: Rollin' up!"

            self.move_to_coord(x,y)


            #turn on right motor, slowly, to draw circles left to right
            BrickPi.MotorSpeed[PORT_C] = 120
            #Motor B is off, but let's get it's encoder first
            motor_B.encoder = BrickPi.Encoder[motor_B.port]

            #calculate coordinates continuously until we reach the top, or right side of the canvas
            while 1:
                BrickPiUpdateValues()
                motor_C.encoder = BrickPi.Encoder[motor_C.port]
                x_norm, y_norm = self.coords_from_motor_pos(motor_B.position, motor_C.position)
                print "Currently at", x_norm,y_norm
                if y_norm <= 0: break # reached the top
                if x_norm >= 1: break # reached the right side
                time.sleep(0.02)

            #Good, now move to the next point and roll down.
            x = ((r_min + r_step*(i+1)) ** 2 - self.v_margin ** 2) ** 0.5
            y = self.v_margin

            if x > self.h_margin + self.canvas_size:
                x = self.h_margin + self.canvas_size
                y = ((r_min+r_step*(i+1)) ** 2 - (self.h_margin+self.canvas_size) ** 2) ** 0.5
                print "Phase 2: Moving down"
            else:
                print "Phase 1: Moving down"

            self.move_to_coord(x,y)
            #turn on right motor, slowly to draw circles from right to left.
            BrickPi.MotorSpeed[PORT_C] = -80
            #Motor B is off, but let's get it's encoder first
            motor_B.encoder = BrickPi.Encoder[motor_B.port]

            #calculate coordinates continuously until we reach the top, or right side of the canvas
            while 1:
                BrickPiUpdateValues()
                motor_C.encoder = BrickPi.Encoder[motor_C.port]
                x_norm, y_norm = self.coords_from_motor_pos(motor_B.position, motor_C.position)
                print "Currently at", x_norm,y_norm
                if y_norm >= 1: break # reached the bottom
                if x_norm <= 0: break # reached the left side
                time.sleep(0.02)

        print "Done drawing"

    # Calibration functions
    def lmf(self):
        self.move_motor_for_time(self.drive_motors[0].port, 100)

    def lmb(self):
        self.move_motor_for_time(self.drive_motors[0].port, -100)

    def rmf(self):
        self.move_motor_for_time(self.drive_motors[1].port, 100)

    def rmb(self):
        self.move_motor_for_time(self.drive_motors[1].port, -100)

    def left_fwd(self):
        BrickPi.MotorSpeed[self.drive_motors[0].port] = 100
        BrickPiUpdateValues()

    def left_stop(self):
        BrickPi.MotorSpeed[self.drive_motors[0].port] = 0
        BrickPiUpdateValues()


    @staticmethod
    def move_motor_for_time(port, speed, runtime=0.3):
        numticks = int(runtime/0.03)
        for i in range(numticks):   #Have to repeat this, otherwise output goes to 0.
            BrickPi.MotorSpeed[port] = int(speed)
            BrickPiUpdateValues()
            time.sleep(0.03)
        BrickPi.MotorSpeed[port] = 0
        BrickPiUpdateValues()

    @staticmethod
    def stop_all_motors():
        BrickPi.MotorSpeed[PORT_A] = 0
        BrickPi.MotorSpeed[PORT_B] = 0
        BrickPi.MotorSpeed[PORT_C] = 0
        BrickPi.MotorSpeed[PORT_D] = 0
        BrickPiUpdateValues()