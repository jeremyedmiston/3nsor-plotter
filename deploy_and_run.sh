# !/bin/bash
# Use this script in pycharm as a deploy script to get things running.

RPI="brickpiplus"
SERVER_SCRIPT="plotter.py"

#copy the server script over. 
scp $SERVER_SCRIPT pi@$RPI:~/


# Run the server script
# The < and > magic makes sure it runs in the background so ssh command returns

ssh pi@$RPI "python $SERVER_SCRIPT vid < /dev/null > /tmp/oshiftlogfile 2>&1 &"
sleep 3
echo "Remote server started"
