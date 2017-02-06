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
#   Pen Motor - Port A (!)
# 	Left Motor  - Port B
# 	Right Motor - Port C
#
# PREREQUISITES
#	Tornado Web Server for Python
#   Python PIL
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
import sys
from PIL import Image, ImageDraw

# My own stuff
from ropeplotter import RopePlotter, Logger, Throttler, get_ip_address
from settings import *

#Ev3dev for drawing and buttons
import ev3dev.auto as ev

################## Globals. I know. #################

c = 0               # movement command.
websockets = []     # list of open sockets.


################# Set up web server & threads #####################


# Initialize Tornado to use 'GET' and load index.html
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html", kp=KP, ti=TI, td=TD, ll=L_ROPE_0, lr=R_ROPE_0, sl=SCAN_LINES, aw=ROPE_ATTACHMENT_WIDTH, cm_to_deg=CM_TO_DEG)


class UploadHandler(tornado.web.RequestHandler):
    def post(self):
        #print(self.request.files)
        if 'file_0' in self.request.files:
            fileinfo = self.request.files['file_0'][0]
            fname = fileinfo['filename']
            extension = os.path.splitext(fname)[1]

            if extension == '.jpg' or extension == '.jpeg':
                output_file = open("uploads/picture.jpg", 'wb')
                output_file.write(fileinfo['body'])
                output_file.close()
            elif extension == '.csv':
                output_file = open("uploads/coords.csv", 'wb')
                output_file.write(fileinfo['body'])
                output_file.close()
                coordsfile = open('uploads/coords.csv', 'r')
                file_body = coordsfile.readlines()
                pointlist = []
                for s_coord in file_body:
                    if ',' in s_coord:
                        coord = [float(c) * PREVIEW_SIZE for c in s_coord.split(",")]
                        pointlist += [tuple(coord)]
                coordsfile.close()

                im_result = Image.new("L", (PREVIEW_SIZE, PREVIEW_SIZE), color=200)
                draw = ImageDraw.Draw(im_result)
                draw.line(pointlist, fill=60, width=1)
                del draw
                im_result.save('uploads/preview.jpg')
            else:
                return

            wsSend("file " + fname + " is uploaded")
            self.finish("Done uploading")
        else:
            return


#Code for handling the data sent from the webpage
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global websockets
        if self not in websockets:
            websockets.append(self)
        print('connection opened...')

    def check_origin(self, origin):
        return True

    def on_message(self, message):  # receives the data from the webpage and is stored in the variable message
        global c
        c = json.loads(message)
        #print(c)

    def on_close(self):
        global websockets
        if self in websockets:
            websockets.remove(self)
        print('connection closed...')


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
    (r"/uploads/(.*)", tornado.web.StaticFileHandler, {"path": "./uploads"}),
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
        self.plotter = RopePlotter(L_ROPE_0, R_ROPE_0, ROPE_ATTACHMENT_WIDTH, Kp=KP, Ki=TI, Kd=TD, cm_to_deg=CM_TO_DEG)
        self.throttle = Throttler(MOTOR_CMD_RATE)

    def run(self):
        global c
        print("Starting interaction thread")
        buttons = ev.Button()
        while running:


            if type(c) == dict:
                # We got settings
                if 'kp' in c:
                    #We got pid settings
                    self.plotter.Kp = c['kp']
                    self.plotter.Ti = c['ti']
                    self.plotter.Td = c['td']
                    self.plotter.cm_to_deg = int(c['cm_to_deg'])
                    wsSend("PID parameters set")

                if 'll' in c:
                    #we got rope length settings
                    self.plotter.l_rope_0 = c['ll']
                    self.plotter.r_rope_0 = c['lr']
                    self.plotter.att_dist = c['aw']
                    self.plotter.scanlines = int(c['sl'])
                    wsSend("Plotter settings set")
                c=''

            if c == 'left-fwd' or buttons.right:
                #print "Running left motor fwd"
                self.plotter.left_fwd()

            elif c == 'left-stop' or not (buttons.right and buttons.top):
                #print "Stopping left motor"
                self.plotter.left_stop()
                c = ''

            elif c == 'right-fwd' or buttons.bottom:
                #print "Running right motor forward"
                self.plotter.right_fwd()

            elif c == 'right-back' or buttons.left:
                #print "Running right motor back"
                self.plotter.right_back()

            elif c == 'right-stop' or not (buttons.left and buttons.bottom):
                #print "Stopping right motor"
                self.plotter.right_stop()
                c = ''

            elif c == 'left-back' or buttons.top:
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
                    wsSend("[ {0:.2f}V ] Plot {1:.2f}% done".format(self.plotter.battery.measured_voltage/1000000.0, pct_done))
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
            elif c == 'plotwaves':
                wsSend("Plotting waves")
                # c stays 'plot' until another command is sent trough the socket
                plot_action = self.plotter.plot_circle_waves()
                c = 'plotting'

            if buttons.backspace:
                sys.exit()
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
    print("Starting web server at {0}:9093".format(get_ip_address()))
    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:  # Triggered by pressing Ctrl+C. Time to clean up.
        #Stop motor thread
        running = False
        #Close all sockets
        for ws in websockets:
            ws.close()
        print("Motor thread stopped")
    except:
        print("Unexpected error:", sys.exc_info()[0])
        raise
