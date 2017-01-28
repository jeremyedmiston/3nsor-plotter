__author__ = 'anton'

import time

from PIL import Image

from ropeplotter.robot_helpers import PIDMotor, clamp, BrickPiPowerSupply
import ev3dev.auto as ev3

UP = 0      # pen up
DOWN = -25
SLOW = 110
FAST = 220

class RopePlotter(object):
    def __init__(self, l_rope_0, r_rope_0, attachment_distance, cm_to_deg=180, Kp=2.2, Ki=0.2, Kd=0.02, Kp_neg_factor=.5, max_spd=800):
        if ev3.current_platform == 'brickpi':
            self.battery = BrickPiPowerSupply()
            factor = 2
        else:
            self.battery = ev3.PowerSupply()
            factor = 1

        self.__l_rope_0 = float(l_rope_0)
        self.__r_rope_0 = float(r_rope_0)
        self.__att_dist = float(attachment_distance)
        self.direction = 1 # -1 is for reversing motors


        self.cm_to_deg = cm_to_deg * factor

        # Calculate the height of triangle made up by the two ropes
        self.v_margin = self.triangle_area(self.__l_rope_0, self.__r_rope_0, self.__att_dist) / self.__att_dist * 2

        # Using pythagoras to find distance from bottom triangle point to left doorframe
        self.h_margin = (self.__l_rope_0 ** 2 - self.v_margin ** 2) ** 0.5

        # For convenience, the canvas is square and centered between the attachment points
        self.canvas_size = self.__att_dist - 2 * self.h_margin

        # Start the engines
        self.pen_motor = PIDMotor(ev3.OUTPUT_A, Kp=1.5, Ki=1, Kd=0.05, brake=0.3, max_spd=100, verbose=True, speed_reg=False)
        self.pen_motor.positionPID.precision = 4
        self.pen_motor.speedPID.Kp = 0.04
        self.left_motor = PIDMotor(ev3.OUTPUT_B, Kp=Kp, Ki=Ki, Kd=Kd, max_spd=max_spd)
        self.right_motor = PIDMotor(ev3.OUTPUT_C, Kp=Kp, Ki=Ki, Kd=Kd, max_spd=max_spd)

        # Build lists for iterating over all motors
        self.drive_motors = [self.left_motor, self.right_motor]
        self.all_motors = [self.left_motor, self.right_motor, self.pen_motor]

        # Set starting point
        self.set_control_zeroes()




    # Getters & setters for plotter properties. Python style, Baby!
    # After setting these, some calculations need to be done, that's why I define special functions
    # And decorate them as setters and getters.

    @property
    def Kp(self):
        return self.drive_motors[0].positionPID.Kp

    @Kp.setter
    def Kp(self,Kp):
        for motor in self.drive_motors:
            motor.positionPID.Kp = float(Kp)

    @property
    def Ti(self):
        return self.drive_motors[0].positionPID.Ti

    @Ti.setter
    def Ti(self, Ti):
        for motor in self.drive_motors:
            motor.positionPID.Ti = float(Ti)

    @property
    def Td(self):
        return self.drive_motors[0].positionPID.Td

    @Td.setter
    def Td(self, Td):
        for motor in self.drive_motors:
            motor.positionPID.Td = float(Td)

    @property
    def max_speed(self):
        return self.drive_motors[0].positionPID.max_out

    @max_speed.setter
    def max_speed(self, n):
        for motor in self.drive_motors:
            motor.positionPID.max_out = int(n)

    @property
    def l_rope_0(self):
        return self.__l_rope_0

    @l_rope_0.setter
    def l_rope_0(self, length):
        self.__l_rope_0 = float(length)
        self.calc_constants()

    @property
    def r_rope_0(self):
        return self.__r_rope_0

    @r_rope_0.setter
    def r_rope_0(self, length):
        self.__r_rope_0 = float(length)
        self.calc_constants()

    @property
    def att_dist(self):
        return self.__att_dist

    @att_dist.setter
    def att_dist(self,length):
        self.__att_dist = float(length)
        self.calc_constants()

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

    ### Calculations for global (doorframe) to local (canvas) coordinates and back. ###
    def motor_targets_from_norm_coords(self,x_norm, y_norm):
        x,y = self.normalized_to_global_coords(x_norm,y_norm)
        return self.motor_targets_from_coords(x,y)

    def motor_targets_from_coords(self,x, y):
        l_rope = (x ** 2 + y ** 2) ** 0.5
        r_rope = ((self.__att_dist - x) ** 2 + y ** 2) ** 0.5
        l_target = (l_rope - self.__l_rope_0) * self.cm_to_deg
        r_target = (r_rope - self.__r_rope_0) * self.cm_to_deg

        return int(l_target), int(r_target)

    def coords_from_motor_pos(self,l_motor,r_motor):
        l_rope = l_motor / self.cm_to_deg + self.__l_rope_0
        r_rope = r_motor / self.cm_to_deg + self.__r_rope_0
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

    ### Movement functions ###
    def set_control_zeroes(self):
        for motor in self.all_motors:
            motor.position = 0
            #motor.positionPID.zero = motor.position

    def move_to_coord(self,x,y):
        motor_b_target, motor_c_target  = self.motor_targets_from_coords(x, y)
        self.move_to_targets((motor_b_target, motor_c_target))

    def move_to_norm_coord(self, x_norm, y_norm):
        motor_b_target, motor_c_target = self.motor_targets_from_norm_coords(x_norm, y_norm)
        #print "Moving to ", x_norm, ",", y_norm, "(At", motor_b_target, motor_c_target, ")"
        self.move_to_targets((motor_b_target, motor_c_target))

    def move_to_targets(self, targets):
        # Set targets
        for motor, tgt in zip(self.drive_motors, targets):
            motor.position_sp = tgt
        # Now wait for the motors to reach their targets
        # Alas ev3dev's run_to_abs_pos is not usable. Have to use my own PID controller.

        while 1:
            for motor in self.drive_motors:
                motor.run()

            if all([motor.positionPID.target_reached for motor in self.drive_motors]):
                self.left_motor.stop()
                self.right_motor.stop()
                break

            #We're done calculating and setting all motor speeds!
            time.sleep(0.02)

    ### Advanced plotting functions by chaining movement functions ###
    def test_drive(self):
        # A little disturbance in the force
        self.move_to_norm_coord(0.1,0.1)
        self.move_to_norm_coord(0.5,0.5)
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

    def plot_circle_waves(self, num_circles=20):

        im = Image.open("uploads/picture.jpg").convert("L")
        w, h = im.size
        pixels = im.load()

        r_min = (self.h_margin ** 2 + self.v_margin ** 2) ** 0.5
        r_max = ((self.h_margin + self.canvas_size) ** 2 + (self.v_margin + self.canvas_size) ** 2) ** 0.5
        r_step = (r_max - r_min) / num_circles
        anchor_motor, drive_motor = self.drive_motors


        # First draw circles with left anchor point as center.
        for i in range(1, num_circles, 2):
            # Move to the starting point at x,y
            # Calculate where a circle with radius r_min+r_step*i crosses the left margin.
            x = self.h_margin
            y = ((r_min + r_step * i) ** 2 - self.h_margin ** 2) ** 0.5  # This is the same left and right
            if y > self.v_margin + self.canvas_size:
                # We reached the bottom, now we check where circles cross the bottom margin
                x = ((r_min + r_step * i) ** 2 - (self.v_margin + self.canvas_size) ** 2) ** 0.5
                y = self.v_margin + self.canvas_size  # This is the same left and right
            self.move_to_coord(x, y)
            anchor_line = anchor_motor.position
            direction = 1

            # Now calculate coordinates continuously until we reach the top, or right side of the canvas
            drive_motor.duty_cycle_sp = 40
            while 1:
                # Look at the pixel we're at and move pen up or down accordingly
                x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position,
                                                            self.drive_motors[1].position)
                pixel_location = (clamp(x_norm * w, (0, w - 1)), clamp(y_norm * w, (0, h - 1)))

                if anchor_motor.position > anchor_line + 30:
                    direction = -1
                elif anchor_motor.position < anchor_line -30:
                    direction = 1

                #anchor_motor.duty_cycle_sp = int((pixels[pixel_location]-255)/3) * direction #Move fast if the pixel is dark.



                if y_norm <= 0:
                    break  # reached the top
                if x_norm >= 1:
                    break  # reached the right side


            drive_motor.stop()

            # Yield to allow pause/stop and show percentage completion
            yield (i * 50.0) / num_circles * 0.66

            # Good, now move to the next point and roll down.
            x = ((r_min + r_step * (i + 1)) ** 2 - self.v_margin ** 2) ** 0.5
            y = self.v_margin

            if x > (self.h_margin + self.canvas_size):  # Reached right side
                x = self.h_margin + self.canvas_size
                y = ((r_min + r_step * (i + 1)) ** 2 - (self.h_margin + self.canvas_size) ** 2) ** 0.5

            self.move_to_coord(x, y)

            drive_motor.duty_cycle_sp = -20
            # Calculate coordinates continuously until we reach the top, or right side of the canvas
            while 1:
                # Look at the pixel we're at and move pen up or down accordingly
                x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position,
                                                            self.drive_motors[1].position)
                pixel_location = (int(clamp(x_norm * w, (0, w - 1))), int(clamp(y_norm * w, (0, h - 1))))

                if anchor_motor.position > anchor_line + 30:
                    direction = -1
                elif anchor_motor.position < anchor_line - 30:
                    direction = 1

                #anchor_motor.duty_cycle_sp = int(
                #    (pixels[pixel_location] - 255) / 3) * direction  # Move fast if the pixel is dark.

                if y_norm >= 1:
                    break  # reached the bottom
                if x_norm <= 0:
                    break  # reached the left side
                time.sleep(0.02)

            drive_motor.stop()

            # Yield to allow pause/stop and show percentage
            yield ((i + 1) * 50.0) / num_circles * 0.66

    def plot_circles(self, num_circles=20):


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
                # Calculate where a circle with radius r_min+r_step*i crosses the left margin.
                x = self.h_margin + (right_side_mode * self.canvas_size)
                y = ((r_min+r_step*i)**2 - self.h_margin ** 2) ** 0.5   # This is the same left and right
                if y > self.v_margin+self.canvas_size:
                    # We reached the bottom, now we check where circles cross the bottom margin
                    if right_side_mode:
                        x = self.h_margin*2 + self.canvas_size - ((r_min + r_step*i) ** 2 - (self.v_margin + self.canvas_size) ** 2) ** 0.5
                    else:
                        x = ((r_min + r_step*i) ** 2 - (self.v_margin + self.canvas_size) ** 2) ** 0.5
                    y = self.v_margin+self.canvas_size  # This is the same left and right
                self.move_to_coord(x,y)

                # Now calculate coordinates continuously until we reach the top, or right side of the canvas
                # Motor B is off, so let's get it's encoder only once
                while 1:
                    # Look at the pixel we're at and move pen up or down accordingly
                    x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position, self.drive_motors[1].position)
                    pixel_location = (clamp(x_norm * w, (0,w-1)), clamp(y_norm * w, (0,h-1)))
                    if pixels[pixel_location] < 60 + 60 * right_side_mode:
                        self.pen_motor.position_sp = DOWN
                        if not self.pen_motor.positionPID.target_reached: drive_motor.stop()
                        #if not DOWN-3 < self.pen_motor.position < DOWN + 3: drive_motor.stop()
                        self.pen_motor.run_to_abs_pos()
                        #turn on motors in different direction to draw horizontalish lines
                        drive_motor.run_at_speed_sp(SLOW)
                    else:
                        self.pen_motor.run_to_abs_pos(position_sp=UP)
                        #turn on motors in different direction to draw horizontalish lines
                        drive_motor.run_at_speed_sp(FAST)

                    if y_norm <= 0:
                        break # reached the top
                    if (not right_side_mode) and x_norm >= 1:
                        break # reached the right side
                    if right_side_mode and x_norm <= 0:
                        break

                drive_motor.stop()
                # Pen up
                self.pen_motor.run_to_abs_pos(position_sp=UP)

                # Yield to allow pause/stop and show percentage
                yield (i*50.0+right_side_mode*50.0)/num_circles * 0.66

                #Good, now move to the next point and roll down.
                if right_side_mode:
                    x = self.h_margin*2 + self.canvas_size - ((r_min + r_step*(i+1)) ** 2 - self.v_margin ** 2) ** 0.5

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

                # Calculate coordinates continuously until we reach the top, or right side of the canvas
                while 1:
                    # Look at the pixel we're at and move pen up or down accordingly
                    x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position, self.drive_motors[1].position)
                    pixel_location = (int(clamp(x_norm * w, (0,w-1))), int(clamp(y_norm * w, (0,h-1))))

                    if pixels[pixel_location] < 60 + 60 * right_side_mode: # About 33% gray
                        self.pen_motor.position_sp = DOWN
                        if not self.pen_motor.positionPID.target_reached: drive_motor.stop()
                        self.pen_motor.run_to_abs_pos()
                        drive_motor.run_at_speed_sp(-SLOW)
                    else:
                        self.pen_motor.run_to_abs_pos(position_sp=UP)
                        drive_motor.run_at_speed_sp(-FAST)

                    if y_norm >= 1:
                        break # reached the bottom
                    if x_norm <= 0 and not right_side_mode:
                        break # reached the left side
                    if x_norm >= 1 and right_side_mode:
                        break
                    time.sleep(0.02)

                drive_motor.stop()
                self.pen_motor.run_to_abs_pos(position_sp=UP)

                # Yield to allow pause/stop and show percentage
                yield ((i+1)*50.0+right_side_mode*50.0)/num_circles * 0.66

        # Now draw horizontal lines.
        for i in range(0,num_circles, 2):
            x = self.h_margin
            y = self.v_margin + i * self.canvas_size * 1.0 / num_circles
            self.move_to_coord(x,y)
            while 1:
                    # Look at the pixel we're at and move pen up or down accordingly
                    x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position, self.drive_motors[1].position)
                    pixel_location = (clamp(x_norm * w, (0,w-1)), clamp(y_norm * w, (0,h-1)))
                    if pixels[pixel_location] < 160:
                        self.pen_motor.position_sp = DOWN
                        if not self.pen_motor.positionPID.target_reached:
                            self.right_motor.stop()
                            self.left_motor.stop()
                        self.pen_motor.position_sp()
                        #turn on motors in different direction to draw horizontalish lines
                        self.right_motor.run_at_speed_sp(SLOW)
                        self.left_motor.run_at_speed_sp(-SLOW)
                    else:
                        self.pen_motor.position_sp(position_sp=UP)
                        #turn on motors in different direction to draw horizontalish lines
                        self.right_motor.run_at_speed_sp(FAST)
                        self.left_motor.run_at_speed_sp(-FAST)
                    if x_norm >= 1:
                        break

            self.right_motor.stop()
            self.left_motor.stop()
            # Pen up
            self.pen_motor.run_to_abs_pos(position_sp=UP)
            yield 66 + i * 33.33 / num_circles

            x = self.h_margin + self.canvas_size
            y = self.v_margin + (i+1) * self.canvas_size * 1.0 / num_circles
            self.move_to_coord(x,y)
            while 1:

                    # Look at the pixel we're at and move pen up or down accordingly
                    x_norm, y_norm = self.coords_from_motor_pos(self.drive_motors[0].position, self.drive_motors[1].position)
                    pixel_location = (clamp(x_norm * w, (0,w-1)), clamp(y_norm * w, (0,h-1)))
                    if pixels[pixel_location] < 160:
                        self.pen_motor.run_to_abs_pos(position_sp=DOWN)
                        #turn on motors in different direction to draw horizontalish lines
                        self.right_motor.run_at_speed_sp(-SLOW)
                        self.left_motor.run_at_speed_sp(SLOW)
                    else:
                        self.pen_motor.run_to_abs_pos(position_sp=UP)
                        self.right_motor.run_at_speed_sp(-FAST)
                        self.left_motor.run_at_speed_sp(FAST)

                    if x_norm <= 0:
                        break

            self.right_motor.stop()
            self.left_motor.stop()
            # Pen up
            self.pen_motor.run_to_abs_pos(position_sp=UP)
            yield 66 + (i+1) * 33.33 / num_circles

        self.move_to_norm_coord(0,0)


    ### Calibration & manual movement functions ###
    def pen_up(self):
        self.pen_motor.run_to_abs_pos(position_sp=UP)

    def pen_down(self):
        self.pen_motor.run_to_abs_pos(position_sp=DOWN)

    def left_fwd(self):
        self.left_motor.run_direct(duty_cycle_sp=50)

    def left_stop(self):
        self.left_motor.stop()

    def left_back(self):
        self.left_motor.run_direct(duty_cycle_sp=-50)

    def right_fwd(self):
        self.right_motor.run_direct(duty_cycle_sp=50)

    def right_stop(self):
        self.right_motor.stop()

    def right_back(self):
        self.right_motor.run_direct(duty_cycle_sp=-50)

    def stop_all_motors(self):
        for motor in self.all_motors:
            motor.stop()
        print("Motors stopped")





