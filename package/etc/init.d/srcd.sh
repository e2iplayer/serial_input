#!/bin/sh
### BEGIN INIT INFO
# Provides: serial input
# Required-Start:
# Required-Stop:
# Default-Start:     S
# Default-Stop:
### END INIT INFO

/usr/bin/killall srcd_respawner.sh
/usr/bin/killall srcd
/sbin/srcd_respawner.sh &

