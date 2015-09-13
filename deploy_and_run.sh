#!/usr/bin/env bash
# Use this script in pycharm as a deploy script to get things running.

RPI="brickpiplus"
SERVER_SCRIPT="plotserver.py"

#copy the server script over. 
#scp $SERVER_SCRIPT pi@$RPI:~/
scp *.py pi@$RPI:~/Plotter/
scp *.html pi@$RPI:~/Plotter/

# Run the server script
# The < and > magic makes sure it runs in the background so ssh command returns

ssh pi@$RPI "python Plotter/$SERVER_SCRIPT < /dev/null > /tmp/plotterlog 2>&1 &"
sleep 3
echo "Remote server started"
