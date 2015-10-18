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
#   Pen Motor - Port D (!)
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
#	If the error does not go away, try changing the port number '9093' both in the client and server code


####################### Imports #########################


# To run motors on the brickpi, in a separate thread
import threading

# Webserver
import tornado.ioloop
import tornado.web
import tornado.websocket
import tornado.template
import json,os

# My own stuff
from ropeplotter import RopePlotter, Logger, Throttler, get_ip_address
from settings import *

################## Globals. I know. #################

c = 0               # movement command.
websockets = []     # list of open sockets.


################# Set up web server & threads #####################


# Initialize Tornado to use 'GET' and load index.html
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        #loader = tornado.template.Loader(".")
        #self.write(loader.load("index.html").generate())
        self.render("index.html", kp=KP, ti=TI, td=TD, ll=L_ROPE_0, lr=R_ROPE_0, aw=ROPE_ATTACHMENT_WIDTH)


class UploadHandler(tornado.web.RequestHandler):
    def post(self):
        if self.request.files:
            uploaded_file = self.request.files.values()[0][0]    #Get one file only.
            original_fname = uploaded_file['filename']
            extension = os.path.splitext(original_fname)[1]
            if extension == '.jpg':
                final_filename = "picture.jpg"
            elif extension == '.csv':
                final_filename = "coords.csv"
            else:
                return

            output_file = open("uploads/" + final_filename, 'w')
            output_file.write(uploaded_file['body'])
            wsSend("file " + final_filename + " is uploaded")
            self.finish("file" + final_filename + " is uploaded")
        else:
            return


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
        c = json.loads(message)

    def on_close(self):
        global websockets
        if self in websockets:
            websockets.remove(self)
        print 'connection closed...'


def wsSend(message):
    for ws in websockets:
        # Prepend voltage before each message
        # TODO make a separate voltage gauge on the web page.
        ws.write_message(message)


application = tornado.web.Application([
    (r'/ws', WSHandler),
    (r'/', MainHandler),
    (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": "./static"}),
    (r"/css/(.*)", tornado.web.StaticFileHandler, {"path": "./css"}),
    (r"/fonts/(.*)", tornado.web.StaticFileHandler, {"path": "./fonts"}),
    (r"/logs/(.*)", tornado.web.StaticFileHandler, {"path": "./logs"}),
    (r"/upload", UploadHandler)
])


class MotorThread(threading.Thread):
    """
    This thread interacts with the plotter via the global 'c' which containts plotter commands.
    """

    def __init__(self):
        self.motor_log = Logger("Motors")
        threading.Thread.__init__(self)
        self.plotter = RopePlotter(L_ROPE_0, R_ROPE_0, ROPE_ATTACHMENT_WIDTH, Kp=KP, Ti=TI, Td=TD, maxpower=MAXPWR)
        self.throttle = Throttler(MOTOR_CMD_RATE)

    def run(self):
        global c
        print "Starting motor thread"
        while running:
            if type(c) == dict:
                # We got settings
                if 'kp' in c:
                    #We got pid settings
                    self.plotter.Kp = c['kp']
                    self.plotter.Ti = c['ti']
                    self.plotter.Td = c['td']
                    wsSend("PID parameters set")

                if 'll' in c:
                    #we got rope length settings
                    self.plotter.l_rope_0 = c['ll']
                    self.plotter.r_rope_0 = c['lr']
                    self.plotter.att_dist = c['aw']
                    wsSend("Plotter settings set")
                c=''

            if c == 'left-fwd':
                #print "Running left motor fwd"
                self.plotter.left_fwd()

            elif c == 'left-stop':
                #print "Stopping left motor"
                self.plotter.left_stop()
                c = ''

            elif c == 'right-fwd':
                #print "Running right motor forward"
                self.plotter.right_fwd()

            elif c == 'right-back':
                #print "Running right motor back"
                self.plotter.right_back()

            elif c == 'right-stop':
                #print "Stopping right motor"
                self.plotter.right_stop()
                c = ''

            elif c == 'left-back':
                #print "Running left motor back"
                self.plotter.left_back()

            elif c == 'pu':
                #print "Pulling pen up"
                self.plotter.pen_up()
                c = ''

            elif c == 'pd':
                #print "Putting pen down"
                self.plotter.pen_down()
                c = ''

            elif c == 'stop':
                self.plotter.left_stop()
                self.plotter.right_stop()
                c = ''
                #print "Stopped"

            elif c == 'testdrive':
                self.plotter.test_drive()
                c = ''

            elif c == 'plot':
                # c stays 'plot' until another command is sent trough the socket
                plot_action = self.plotter.plot_from_file('uploads/coords.csv')
                c = 'plotting'

            elif c == 'plotting':
                try:
                    pct_done = next(plot_action)
                    wsSend("[ {0:.2f}V ] Plot {1:.2f} Percent done".format(self.plotter.get_voltage(),pct_done))
                except StopIteration:
                    c = ''
                    wsSend("Done plotting")

            elif c == 'zero':
                wsSend("zero motor positions")
                self.plotter.set_control_zeroes()
                c = ''
            elif c == 'plotcircles':
                wsSend("Plotting circles")
                # c stays 'plot' until another command is sent trough the socket
                plot_action = self.plotter.plot_circles()
                c = 'plotting'


            #BrickPiUpdateValues()  # BrickPi updates the values for the motors
            self.throttle.throttle()  #Don't go too fast.

        #Stopped running. Shutting down all motors.
        self.plotter.stop_all_motors()


################## Main #############################

if __name__ == "__main__":
    # Start motor thread
    running = True
    motor_thread = MotorThread()
    motor_thread.setDaemon(True)
    motor_thread.start()

    #set up web server
    application.listen(9093)  # starts the web sockets connection
    print "Starting web server at {0}:9093".format(get_ip_address('wlan0'))
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:  # Triggered by pressing Ctrl+C. Time to clean up.
        #Stop motor thread
        running = False
        #Close all sockets
        for ws in websockets:
            ws.close()
        print "Motor thread stopped"
