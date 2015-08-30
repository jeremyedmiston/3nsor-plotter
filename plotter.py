__author__ = 'anton'

#!/usr/bin/env python
# ##############################################################################################################
# Program Name: plotter
# ================================
# This code is for controlling a plotter by a web browser using web sockets
# History
# ------------------------------------------------
# Author
# Anton Vanhoucke
#
# These files have been made available online through a Creative Commons Attribution-ShareAlike 3.0  license.
# (http://creativecommons.org/licenses/by-sa/3.0/)
#
# ##############################################################################################################

# CONNECTIONS-
# Pen Motor - Port A
# 	Left Motor  - Port B
# 	Right Motor - Port C
#
# PREREQUISITES
#	Tornado Web Server for Python
#
# TROUBLESHOOTING:
#	Don't use Ctrl+Z to stop the program, use Ctrl+c.
#	If you use Ctrl+Z, it will not close the socket and you won't be able to run the program the next time.
#	If you get the following error:
#		"socket.error: [Errno 98] Address already in use "
#	Run this on the terminal:
#		"sudo netstat -ap |grep :9093"
#	Note down the PID of the process running it
#	And kill that process using:
#		"kill pid"
#	If it does not work use:
#		"kill -9 pid"
#	If the error does not go away, try changin the port number '9093' both in the client and server code


####################### Imports #########################


# To run motors on the brickpi, in a separate thread
from BrickPi import *  # import BrickPi.py file to use BrickPi operations
import threading

# Webserver
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.template

# My own stuff
from brickpi_helpers import *

################### Settings ################


MOTOR_CMD_RATE = 20  # Max number of motor commands per second
L_ROPE_0 = 63  # Length of left rope in cm when pen is at 0,0 (top left)
R_ROPE_0 = 95  # same for right tope
ROPE_ATTACHMENT_WIDTH = 90  # space between the two attachment points of the plotter.In my case: door width. In cm.
PULLEY_DIAMETER = 4.4


################## Globals. I know. #################

c = 0  # movement command.
websockets = []  # list of open sockets.
drive_motors = []

################# Movement functions ######################

def pen_up():
    BrickPi.MotorSpeed[PORT_A] = 10
    BrickPiUpdateValues()
    time.sleep(0.3)
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPiUpdateValues()


def pen_down():
    BrickPi.MotorSpeed[PORT_A] = -10
    BrickPiUpdateValues()
    time.sleep(0.3)
    BrickPi.MotorSpeed[PORT_A] = 0
    BrickPiUpdateValues()


def set_motor_zero():
    global drive_motors
    for motor in drive_motors:
        motor.zero = int(BrickPi.Encoder[motor.port])
        print "Encoder zero position set to", motor.zero, "For motor at port:", motor.port


def move_to_coord(x,y,plotter):
    motor_b_target, motor_c_target  = plotter.motor_targets_from_coords(x, y)
    print "Moving to ", x, ",", y, "(At", motor_b_target, motor_c_target, ")"
    move_to_targets((motor_b_target, motor_c_target))


def move_to_norm_coord(x_norm, y_norm, plotter):
    motor_b_target, motor_c_target = plotter.motor_targets_from_norm_coords(x_norm, y_norm)
    print "Moving to ", x_norm, ",", y_norm, "(At", motor_b_target, motor_c_target, ")"
    move_to_targets((motor_b_target, motor_c_target))


def move_to_targets(targets):
    global drive_motors

    # set targets
    for i in range(2):
        drive_motors[i].target = targets[i]

    # run motors until targets are reached
    while 1:
        BrickPiUpdateValues()  # Ask BrickPi to update values for sensors/motors
        for motor in drive_motors:
            #get motor positions
            motor.encoder = BrickPi.Encoder[motor.port]

            #set motor speed accordingly
            speed = motor.get_power()
            BrickPi.MotorSpeed[motor.port] = speed

        if (abs(motor_C.error) < 3) and (abs(motor_B.error) < 3):
            #we got where we want to be. Time for the next coordinate.
            for motor in drive_motors:
                BrickPi.MotorSpeed[motor.port] = 0
            BrickPiUpdateValues()
            break

        #We're done calculating and setting all motor speeds!
        time.sleep(0.02)




def plot_from_file(filename, plotter):
    coords = open(filename)
    num_coords = int(coords.readline())  #coords contains it's length on the first line.
    print num_coords

    #drive to the first coordinate
    pen_up()
    # read from file
    x_norm, y_norm = [float(n) for n in coords.readline().split(",")]
    #move
    move_to_norm_coord(x_norm, y_norm, plotter)
    pen_down()
    for i in range(num_coords - 1):
        # read from file
        x_norm, y_norm = [float(n) for n in coords.readline().split(",")]
        #move
        move_to_norm_coord(x_norm, y_norm, plotter)

    coords.close()


def plot_circles(plotter, num_circles=12):
    r_min = (plotter.h_margin**2+plotter.v_margin**2)**0.5
    r_max = ((plotter.h_margin+plotter.canvas_size)**2 + (plotter.v_margin+plotter.canvas_size)**2)**0.5
    r_step = (r_max-r_min)/num_circles

    for i in range(1, num_circles, 2):
        x = plotter.h_margin
        y = ((r_min+r_step*i)**2 - plotter.h_margin ** 2) ** 0.5
        print "y:",y, "vm:",plotter.v_margin,"rmin:",r_min,"step:",r_step
        if y > plotter.v_margin+plotter.canvas_size:
            x = ((r_min + r_step*i) ** 2 - (plotter.v_margin + plotter.canvas_size) ** 2) ** 0.5
            y = plotter.v_margin+plotter.canvas_size
            print "Phase 3: Rollin' up!"
        else:
            print "Phase 1: Rollin' up!"

        move_to_coord(x,y,plotter)


        #turn on right motor, slowly, to draw circles left to right
        BrickPi.MotorSpeed[PORT_C] = -80
        #Motor B is off, but let's get it's encoder first
        motor_B.encoder = BrickPi.Encoder[motor_B.port]

        #calculate coordinates continuously until we reach the top, or right side of the canvas
        while 1:
            BrickPiUpdateValues()
            motor_C.encoder = BrickPi.Encoder[motor_C.port]
            x_norm, y_norm = plotter.coords_from_motor_pos(motor_B.position, motor_C.position)
            print "Currently at", x_norm,y_norm
            if y_norm <= 0: break # reached the top
            if x_norm >= 1: break # reached the right side
            time.sleep(0.02)

        #Good, now move to the next point and roll down.
        x = ((r_min + r_step*(i+1)) ** 2 - plotter.v_margin ** 2) ** 0.5
        y = plotter.v_margin

        if x > plotter.h_margin + plotter.canvas_size:
            x = plotter.h_margin + plotter.canvas_size
            y = ((r_min+r_step*(i+1)) ** 2 - (plotter.h_margin+plotter.canvas_size) ** 2) ** 0.5
            print "Phase 2: Moving down"
        else:
            print "Phase 1: Moving down"

        move_to_coord(x,y,plotter)
        #turn on right motor, slowly to draw circles from right to left.
        BrickPi.MotorSpeed[PORT_C] = 80
        #Motor B is off, but let's get it's encoder first
        motor_B.encoder = BrickPi.Encoder[motor_B.port]

        #calculate coordinates continuously until we reach the top, or right side of the canvas
        while 1:
            BrickPiUpdateValues()
            motor_C.encoder = BrickPi.Encoder[motor_C.port]
            x_norm, y_norm = plotter.coords_from_motor_pos(motor_B.position, motor_C.position)
            print "Currently at", x_norm,y_norm
            if y_norm >= 1: break # reached the bottom
            if x_norm <= 0: break # reached the left side
            time.sleep(0.02)

    print "Done drawing"


################# Set up web server & threads #####################


# Initialize Tornado to use 'GET' and load index.html
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        loader = tornado.template.Loader(".")
        self.write(loader.load("index.html").generate())


#Code for handling the data sent from the webpage
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global websockets
        if self not in websockets:
            websockets.append(self)
        print 'connection opened...'

    def check_origin(self, origin):
        return True

    def on_message(self, message):  # receives the data from the webpage and is stored in the variable message
        global c
        print 'received:', message  # prints the revived from the webpage
        c = message
        print c

    def on_close(self):
        global websockets
        if self in websockets:
            websockets.remove(self)
        print 'connection closed...'


def wsSend(message):
    for ws in websockets:
        pass
        # ws.write_message(message)


application = tornado.web.Application([
    (r'/ws', WSHandler),
    (r'/', MainHandler),
    (r"/(.*)", tornado.web.StaticFileHandler, {"path": "./resources"}),
])


class MotorThread(threading.Thread):
    def __init__(self, plotter):
        self.motor_log = Logger("Motors")
        threading.Thread.__init__(self)
        self.plotter = plotter
        self.throttle = Throttler(MOTOR_CMD_RATE)

    def run(self):
        global c
        print "Starting motor thread"
        while running:

            if c == 'lmf':
                print "Running left motor fwd"
                BrickPi.MotorSpeed[PORT_B] = 100  #Set the speed of MotorA (-255 to 255)

            elif c == 'lmb':
                print "Running left motor back"
                BrickPi.MotorSpeed[PORT_B] = -100
            elif c == 'rmf':
                print "Running right motor forward"
                BrickPi.MotorSpeed[PORT_C] = 100
            elif c == 'rmb':
                print "Running right motor back"
                BrickPi.MotorSpeed[PORT_C] = -100
            elif c == 'stop':
                print "Stopped"
                BrickPi.MotorSpeed[PORT_B] = 0
                BrickPi.MotorSpeed[PORT_C] = 0
            elif c == 'plot':
                wsSend("Start plotting...")
                plot_from_file('coords.csv', self.plotter)
                print "Done plotting"
                c = ''
            elif c == 'zero':
                wsSend("zero motor positions")
                set_motor_zero()
                c = ''
            elif c == 'plotcircles':
                wsSend("Plotting circles")
                plot_circles(self.plotter)
                c = ''

            BrickPiUpdateValues()  # BrickPi updates the values for the motors
            self.throttle.throttle()  #Don't go too fast.


################## Main #############################

if __name__ == "__main__":
    # Set up logging
    server_log = Logger("Server")



    #  Setup BrickPi and motors
    server_log.log("Revving up engines")
    BrickPiSetup()  # setup the serial port for communication
    BrickPi.MotorEnable[PORT_A] = 1  #Enable the Motor A
    BrickPi.MotorEnable[PORT_B] = 1  #Enable the Motor B
    BrickPi.MotorEnable[PORT_C] = 1  #Enable the Motor C
    BrickPi.MotorEnable[PORT_D] = 1  #Enable the Motor D

    motor_B = motorPID_control(PORT_B)
    motor_C = motorPID_control(PORT_C)
    drive_motors = [motor_B, motor_C]
    no_values = 1
    while no_values:
        # Make sure we have something before we start running
        # So we wait until no_values goes 0, which means values updated OK
        no_values = BrickPiUpdateValues()
    set_motor_zero()

    # Set up plotting installation
    my_plotter = plot_installation(L_ROPE_0, R_ROPE_0, ROPE_ATTACHMENT_WIDTH)

    # Start motor thread
    running = True
    thread1 = MotorThread(my_plotter)
    thread1.setDaemon(True)
    thread1.start()

    #set up web server

    application.listen(9093)  # starts the websockets connection
    server_log.newline()  #done setting up. Log it.
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:  #Triggered by pressing Ctrl+C. Time to clean up.
        running = False
        #Shutting down all motors.
        BrickPi.MotorSpeed[PORT_A] = 0
        BrickPi.MotorSpeed[PORT_B] = 0
        BrickPi.MotorSpeed[PORT_C] = 0
        BrickPi.MotorSpeed[PORT_D] = 0
        BrickPiUpdateValues()

        print "Motors & motor thread stopped"
