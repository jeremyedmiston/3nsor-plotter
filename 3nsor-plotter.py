#!/usr/bin/env python3

__author__ = 'anton'


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
from PIL import Image, ImageDraw, ImageOps
import logging,time
from io import StringIO, BytesIO

# My own stuff
from ropeplotter import RopePlotter, Throttler, get_ip_address
from settings import *

#Ev3dev for drawing and buttons
import ev3dev.auto as ev

######################### Globals. ################################

c = 0               # movement command.
websockets = []     # list of open sockets.

# instantiate a plotter object
plotter = RopePlotter(L_ROPE_0, R_ROPE_0, ROPE_ATTACHMENT_WIDTH, Kp=KP, Ki=TI, Kd=TD, cm_to_deg=CM_TO_DEG, chalk=CHALK)

logging.basicConfig(filename='plotter.log',
                    format='%(asctime)s.%(msecs)03d - %(funcName)s: %(message)s',
                    datefmt="%H:%M:%S")
plotter_log = logging.getLogger("Plotter")


################# Set up web server & threads #####################


# Initialize Tornado to use 'GET' and load index.html
class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render("index.html",
                    kp=plotter.Kp,
                    ti=plotter.Ti,
                    td=plotter.Td,
                    ll=plotter.l_rope_0,
                    lr=plotter.r_rope_0,
                    rs=plotter.r_step,
                    aw=plotter.att_dist,
                    cm_to_deg=plotter.cm_to_deg)


class UploadHandler(tornado.web.RequestHandler):
    def post(self):
        if 'file_0' in self.request.files:
            fileinfo = self.request.files['file_0'][0]
            fname = fileinfo['filename']
            extension = os.path.splitext(fname)[1]
            logging.debug(extension, extension.upper())
            if extension.upper() == '.JPG' or extension.upper() == '.JPEG':
                plotter_log.debug("started image download")
                img_file = open("uploads/picture.jpg", 'wb')

                plotter_log.debug("started file write")
                img_file.write(fileinfo['body'])

                plotter_log.debug("started closing file")
                img_file.close()

                plotter_log.debug("file closed")
                # im = Image.open(img_file)
                # logging.debug("image opened")
                # im = ImageOps.fit(im,(500, 500), Image.ANTIALIAS)
                # logging.debug("image scaled")
                # im.save("uploads/picture.jpg")
                # logging.debug("image saved")
            if extension.upper() == '.PNG':
                img_file = open("uploads/tmp.png", 'wb')
                img_file.write(fileinfo['body'])
                img_file.close() #is this needed?
                im = Image.open(img_file)
                # im = ImageOps.fit(im, (500, 500), Image.ANTIALIAS)
                im.save("uploads/picture.jpg")
            elif extension.upper() == '.CSV':
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


# Code for handling the data sent from the webpage
class WSHandler(tornado.websocket.WebSocketHandler):
    def open(self):
        global websockets
        if self not in websockets:
            websockets.append(self)
        plotter_log.info('connection opened...')

    def check_origin(self, origin):
        return True

    def on_message(self, message):  # receives the data from the webpage and is stored in the variable message
        global c
        c = json.loads(message)

    def on_close(self):
        global websockets
        if self in websockets:
            websockets.remove(self)
        plotter_log.info('connection closed...')


def wsSend(message):
    for ws in websockets:
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
        threading.Thread.__init__(self)
        self.throttle = Throttler(MOTOR_CMD_RATE)

    def run(self):
        global c, plotter
        buttons = ev.Button()
        right_or_up_pressed_earlier = False
        left_or_down_pressed_earlier = False

        while running:

            if type(c) == dict:
                # We got settings
                if 'kp' in c:
                    #We got pid settings
                    plotter.Kp = c['kp']
                    plotter.Ti = c['ti']
                    plotter.Td = c['td']
                    plotter.cm_to_deg = int(c['cm_to_deg'])
                    wsSend("PID parameters set")

                if 'll' in c:
                    #we got rope length settings
                    plotter.l_rope_0 = c['ll']
                    plotter.r_rope_0 = c['lr']
                    plotter.att_dist = c['aw']
                    plotter.r_step = float(c['rs'])
                    wsSend("Plotter settings set")
                c=''

            # Socket commands
            if c == 'left-fwd':
                plotter.left_fwd()

            elif c == 'left-back':
                plotter.left_back()

            elif c == 'right-fwd':
                plotter.right_fwd()

            elif c == 'right-back':
                plotter.right_back()

            elif buttons.right:
                plotter.left_fwd()
                right_or_up_pressed_earlier = True

            elif buttons.up:
                plotter.left_back()
                right_or_up_pressed_earlier = True

            elif buttons.down:
                plotter.right_fwd()
                left_or_down_pressed_earlier = True

            elif buttons.left:
                plotter.right_back()
                left_or_down_pressed_earlier = True

            elif not (buttons.left and buttons.down) and left_or_down_pressed_earlier:
                plotter.right_stop()
                left_or_down_pressed_earlier = False

            elif not (buttons.right and buttons.up) and right_or_up_pressed_earlier:
                plotter.left_stop()
                right_or_up_pressed_earlier = False

            elif c == 'right-stop':
                plotter.right_stop()
                c = ''

            elif c == 'left-stop':
                plotter.left_stop()
                c = ''

            elif c == 'pu':
                plotter.pen_up()
                c = ''

            elif c == 'pd':
                plotter.pen_down()
                c = ''

            elif c == 'stop':
                plotter.left_stop()
                plotter.right_stop()
                c = ''

            elif c == 'reload':
                plotter.reload_chalk()

            elif c == 'testdrive':
                plotter.test_drive()
                c = ''

            elif c == 'plot':
                # c stays 'plot' until another command is sent trough the socket
                plot_action = plotter.plot_from_file('uploads/coords.csv')
                c = 'plotting'

            elif c == 'plotting':
                try:
                    pct_done = next(plot_action)
                    wsSend("[ {0:.2f}V ] Plot {1:.2f}% done".format(plotter.battery.measured_voltage/1000000.0, pct_done))
                except StopIteration:
                    c = ''
                    wsSend("Done plotting")

            elif c == 'zero':
                wsSend("zero motor positions")
                plotter.set_control_zeroes()
                c = ''
            elif c == 'plotcircles':
                wsSend("Plotting circles")
                # c stays 'plot' until another command is sent trough the socket
                plot_action = plotter.optimized_etch()
                c = 'plotting'
            elif c == 'plotwaves':
                wsSend("Plotting waves")
                # c stays 'plot' until another command is sent trough the socket
                plot_action = plotter.plot_circle_waves()
                c = 'plotting'

            # Button commands
            if buttons.backspace:
                tornado.ioloop.IOLoop.instance().stop()
                # Close all sockets
                for ws in websockets:
                    ws.close()

            self.throttle.throttle()  # Don't go too fast.

        # Stopped running. Shutting down all motors.
        plotter.stop_all_motors()
        plotter_log.info("Socket thread stopped")


################## Main #############################

if __name__ == "__main__":
    # Start motor thread
    running = True
    motor_thread = MotorThread()
    motor_thread.setDaemon(True)
    motor_thread.start()

    # Set logging levels
    if len(sys.argv) > 1: # Whatever argument is enough the lower log levels...
        log_level = logging.DEBUG
    else:
        log_level = logging.CRITICAL

    access_log = logging.getLogger("tornado.access")
    app_log = logging.getLogger("tornado.application")
    gen_log = logging.getLogger("tornado.general")
    access_log.setLevel(log_level)
    app_log.setLevel(log_level)
    gen_log.setLevel(log_level)
    plotter_log.setLevel(log_level)

    # Prepare the screen
    lcd = ev.Screen()
    logo = Image.open('static/logo.jpg')
    img = Image.new("1", (128, 178), color=255)
    img.paste(logo.resize((100, 127)), (14, 0))
    draw = ImageDraw.Draw(img)
    draw.text((2, 127), 'Point your browser to:')
    draw.text((2, 137), '{0}:9093'.format(get_ip_address()))
    draw.text((2, 150), 'press back to exit')
    del draw
    lcd.image.paste(img.rotate(-90), box=(0, 0))

    # Set up web server
    application.listen(9093)  # starts the web sockets connection
    logging.info("Started web server at {0}:9093".format(get_ip_address()))

    # Display ip number on screen for easy connection
    lcd.update()

    try:
        tornado.ioloop.IOLoop.instance().start()
    except KeyboardInterrupt:   # Triggered by pressing Ctrl+C. Time to clean up.
        running = False         # Stop motor thread
        for ws in websockets:   # Close all sockets
            ws.close()

    except:
        logging.critical("Unexpected error:", sys.exc_info()[0])
        raise

    logging.info("Stopped. Bye!")
