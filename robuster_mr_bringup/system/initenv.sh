#!/bin/bash

PASSWD="123456"
WRAPSCRIPT_DIR="/usr/local/share/monitor"
SERVICE_DIR="/usr/lib/systemd/system"

if [[ ! -d "$WRAPSCRIPT_DIR" ]]; then
    echo "monitor dictory doesn't exist.then create it"
    echo "$PASSWD" | sudo -S mkdir $WRAPSCRIPT_DIR
fi

if [[ ! -d "$SERVICE_DIR" ]]; then
    echo "monitor dictory doesn't exist.then create it"
    echo "$PASSWD" | sudo -S mkdir $SERVICE_DIR
fi

echo "$PASSWD" | sudo -S cp ros_wrapper.sh $WRAPSCRIPT_DIR
if [[ ! -x "$WRAPSCRIPT_DIR/ros_wrapper.sh" ]]; then
    echo "Change wrapper file permission."
    echo "$PASSWD" | sudo -S chmod a+x $WRAPSCRIPT_DIR/ros_wrapper.sh
fi

echo "$PASSWD" | sudo -S cp *.service $SERVICE_DIR
echo "$PASSWD" | sudo -S systemctl enable roscore
echo "$PASSWD" | sudo -S systemctl enable robuster_monitor
#echo "$PASSWD" | sudo -S ln -s '/usr/lib/systemd/system/roscore.service' '/etc/systemd/system/roscore.service'
#echo "$PASSWD" | sudo -S ln -s '/usr/lib/systemd/system/robuster_service_monitor.service' '/etc/systemd/system/robuster_service_monitor.service'

echo "$PASSWD" | sudo -S systemctl daemon-reload
echo "$PASSWD" | sudo -S systemctl restart roscore.service
echo "$PASSWD" | sudo -S systemctl restart robuster_monitor.service

