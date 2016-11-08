#!/bin/bash -xe

linuxcnc -r test.ini | grep -A 3 ^axis

