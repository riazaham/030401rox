#!/bin/sh
echo executing compilation and execution...
cd ~/alexdir
gcc alex-pi_working.cpp serialize.cpp serial.cpp -lpthread -lncurses -o a.out
./a.out