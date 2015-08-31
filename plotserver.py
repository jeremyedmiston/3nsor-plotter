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
from rope_plotter import rope_plotter

################### Settings ################


MOTOR_CMD_RATE = 20  # Max number of motor commands per second
L_ROPE_0 = 63  # Length of left rope in cm when pen is at 0,0 (top left)
R_ROPE_0 = 95  # same for right tope
ROPE_ATTACHMENT_WIDTH = 90  # space between the two attachment points of the plotter.In my case: door width. In cm.
PULLEY_DIAMETER = 4.4


################## Globals. I know. #################

c = 0  # movement command.
websockets = []  # list of open sockets.


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
                #BrickPi.MotorSpeed[PORT_B] = 100
                self.plotter.lmf()
            elif c == 'lmb':
                print "Running left motor back"
                #BrickPi.MotorSpeed[PORT_B] = -100
            elif c == 'rmf':
                print "Running right motor forward"
                #BrickPi.MotorSpeed[PORT_C] = 100
            elif c == 'rmb':
                print "Running right motor back"
                #BrickPi.MotorSpeed[PORT_C] = -100
            elif c == 'stop':
                print "Stopped"
                #BrickPi.MotorSpeed[PORT_B] = 0
                #BrickPi.MotorSpeed[PORT_C] = 0
            elif c == 'plot':
                wsSend("Start plotting...")
                self.plotter.plot_from_file('coords.csv')
                print "Done plotting"
                c = ''
            elif c == 'zero':
                wsSend("zero motor positions")
                self.plotter.set_motor_zero()
                c = ''
            elif c == 'plotcircles':
                wsSend("Plotting circles")
                self.plotter.plot_circles(self.plotter)
                c = ''

            BrickPiUpdateValues()  # BrickPi updates the values for the motors
            self.throttle.throttle()  #Don't go too fast.


################## Main #############################

if __name__ == "__main__":
    # Set up logging
    server_log = Logger("Server")



    #  Setup BrickPi and motors
    server_log.log("Revving up engines")

    # Set up plotting installation
    my_plotter = rope_plotter(L_ROPE_0, R_ROPE_0, ROPE_ATTACHMENT_WIDTH)

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
