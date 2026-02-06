#!/bin/sh

# check console
# cat /sys/class/tty/console/active

# Mute
echo "0 4 0 0" |  tee /proc/sys/kernel/printk

# unMute
# echo "7 4 1 7" |  tee /proc/sys/kernel/printk

while true
do
    /sbin/srcd > /dev/null 2>&1
done
