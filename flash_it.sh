#!/bin/bash
avrdude -p m328p -c stk500v1 -b 19200 -P /dev/ttyACM0 -U flash:w:sdisk2.hex
