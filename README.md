# 3nsor - the BrickPi rope plotter #

This plotter is hung by two pieces of rope on a door and controlled by a BrickPi. Since BrickPi has no screen I built
a web interface with tornado. It's the BrickPi version of L3onardo.


## Requirements ##

I had to install tornado manually, as described here http://www.tornadoweb.org/en/stable/. Pip wouldn't compile tornado
right, and the server became real slow.


## How to use ##

1. Clone the repo on the brickpi
2. Start plotter.py
3. Surf to <yourbrickpiaddress>:9093
4. Move the plotter to the origin with the buttons
5. Zero it all
6. Start plotting