#!/bin/sh

stty -F /dev/ttyACM0 -hupcl ospeed 57600 ispeed 57600
exec 3>/dev/ttyACM0

sleep 1
echo "1000,1000,00" >&3
sleep 1
echo "2000,2000,00" >&3
sleep 1
echo "1215,1330,00" >&3
sleep .5
echo "1215,1330,10" >&3
sleep 1
echo "1000,1000,00" >&3
