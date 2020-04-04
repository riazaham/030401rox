#!/bin/sh
echo executing compilation and execution...
cd Alex_NoTLS
gcc alex-pi_final.cpp serialize.cpp serial.cpp -lpthread -lncurses -o alex-pi_final.out
./alex-pi_final.out
