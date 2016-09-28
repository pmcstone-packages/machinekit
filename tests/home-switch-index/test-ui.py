#!/usr/bin/env python

import linuxcnc
import hal

import time
import sys
import os
import math

# unbuffer stdout
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)


def all_joints_homed(joints):
    s.poll()
    for i in range(0,9):
        if joints[i] and not s.homed[i]:
            return False
    return True


def wait_for_home(joints):
    print "homing..."
    start_time = time.time()
    timeout = 10.0
    while (time.time() - start_time) < timeout:
        if all_joints_homed(joints):
            return
        time.sleep(0.1)

    s.poll()
    print "timeout waiting for homing to complete"
    print "s.homed:", s.homed
    print "s.position:", s.position
    # os.system("halcmd show")
    sys.exit(1)



#
# Axes and signals of interest
#
axes = ("X", "Y", "Z")
signals = {"homed" : hal.HAL_BIT,
           "homeswpos" : hal.HAL_FLOAT,
           "hysteresis" : hal.HAL_FLOAT,
           "ixpos" : hal.HAL_FLOAT,
           "ixwidth" : hal.HAL_FLOAT,
           "ixoffset" : hal.HAL_FLOAT,
           }

#
# Stub HAL component for signal access
#
h = hal.component("test-ui")
for a in axes:
    for s, s_type in signals.items():
        h.newpin('%s%s' % (a, s), s_type, hal.HAL_IN)
h.ready()
os.system('halcmd source postgui.hal') # Net above-created pins to signals

#
# connect to LinuxCNC
#

c = linuxcnc.command()
e = linuxcnc.error_channel()
s = linuxcnc.stat()

#
# Start machine
#
c.state(linuxcnc.STATE_ESTOP_RESET)
c.state(linuxcnc.STATE_ON)
c.mode(linuxcnc.MODE_MANUAL)   

#
# Home three axes
#
c.home(0)
c.home(1)
c.home(2)
wait_for_home(joints=[1,1,1,0,0,0,0,0,0])

#
# Print status info and exit
#

def sig(axis, name):
    return h[axis + name]

for a in axes:
    # Home switch below zero homes in negative dir first & the converse
    sign = 1 if sig(a, 'homeswpos') > 0 else -1

    # Calculate home position, incl. hysteresis
    home_position = sig(a, 'homeswpos') - sign * sig(a, 'hysteresis')

    # Calculate index position, incl. width
    index_position = sig(a, 'ixpos') + sign * 0.5 * sig(a, 'ixwidth')

    # Calculate home index offset
    home_index_offset = (home_position - index_position)

    print "axis %s:" % a
    print "    switch pos=%.2f, hyst=%.2f => home pos=%.2f" % \
        (sig(a, 'homeswpos'), sig(a, 'hysteresis'), home_position)
    print "    index pos=%.2f, wid=%.2f => index pos = %.2f" % \
        (sig(a, 'ixpos'), sig(a, 'ixwidth'), index_position)
    print "    home index offset:  calculated = %.2f; actual = %.2f" % \
        (home_index_offset, sig(a, 'ixoffset'))

sys.exit(0)

