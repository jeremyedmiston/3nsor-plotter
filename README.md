# 3nsor - the BrickPi rope plotter #

[![Plotter in action](http://img.youtube.com/vi/YYG3XGfyVHk/0.jpg)](http://www.youtube.com/watch?v=YYG3XGfyVHk)


This plotter is hung by two pieces of rope on a door and controlled by a BrickPi. Since BrickPi has no screen I built
a web interface with tornado. The video is done with the BrickPi branch of this repo. The newest branch is on Ev3dev.

## Hardware requirements ##
- Raspberry Pi with a BrickPi or BrickPi+ shield. The model doesn't matter, model A, B, B+ and v2 all work.
- ...OR an Ev3 brain
- A robot with two ropes attached to motors B and C, and a pen to motor D.
- A small USB wifi stick. Can't leave the robot connected while it's plotting

## Software requirements ##
- **Tornado**
- **Ev3dev** and of course the python bindings.

## Installation ##

1. Get yourself a micro SD with the right Ev3dev image. They exist for both BrickPi and Ev3. http://www.ev3dev.org/docs/getting-started/
2. Install python ev3dev bindings as described here: https://github.com/ddemidov/ev3dev-lang-python
2. Install tornado as described here under *manual*(!) installation here: http://www.tornadoweb.org/en/stable/#installation
2. Clone this repo: `git clone https://github.com/antonvh/3nsor-plotter/`
5. To get battery voltage readings on a BrickPi add `i2c-dev` to `/etc/modules-load.d/modules.conf` and reboot.

## Usage ##

2. Type `cd 3nsor-plotter`. This is important, as tornado expects to be run from it's root.
2. Type `python 3nsor-plotter.py`
3. Surf to `<yourbrickpiaddress>:9093`
4. Move the plotter to the origin with the buttons/keyboard
4. Measure and input rope lengths at origin. (Make sure your paper is in the middle between attachment points.)
5. Zero it all
6. Generate a coords.csv file using the l3onardo script. https://github.com/antonvh/L3onardo-plotter
6. Upload the file
6. Start plotting

## Please fork me ##
And help improve the web interface.

## Good to know ##
- On the Raspberry Pi, My wifi dongle went to sleep all the time, while I was working on this project. I used this to fix it: http://raspberrypi.stackexchange.com/questions/1384/how-do-i-disable-suspend-mode
As long as the server is running, there's no problem.
- The script has virtually no error catching. It will crash if you throw data at it that it is not expecting.

## To do ##
- Increase precision on circle plot drawing
- Build stronger gear box
- Find a way to plot horizontal lines
- Improve rope guiding
- Fix voltage reading over i2c on BrickPi
- Drive slower when drawing, faster when not