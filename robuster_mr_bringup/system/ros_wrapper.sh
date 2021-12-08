#!/bin/bash

echo "Running roslaunch_wrapper <$1> <$2>"
. /home/robuster/robuster_ws/devel/setup.bash

term() {
  echo "Caught SIGTERM signal!"
  kill -TERM "$child" 2>/dev/null
}

trap term SIGTERM



$1 $2 $3 &
EXIT_CODE=$?
child=$!

# wait until all children exits, OR a signal arrive
wait

echo "end of roslaunch_wrapper"

# give some time for child to exits properly
sleep 1

exit $EXIT_CODE
