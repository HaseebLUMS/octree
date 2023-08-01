#!/bin/bash

# 0 for linux, 1 for macos

if [ "$1" = "0" ]; then
    tc qdisc change dev lo root netem rate 200Mbit loss 0% delay 5ms reorder 1%

elif [ "$1" = "1" ]; then
    sudo pfctl -E
    # sudo dnctl pipe 1 config bw 500Mbit/s delay 5 plr 0
    sudo dnctl pipe 1 config plr 0.1
    echo "dummynet out proto udp from any to any pipe 1" | sudo pfctl -f -
    echo "dummynet in proto udp from any to any pipe 1" | sudo pfctl -f -
    echo "dummynet out proto tcp from any to any pipe 1" | sudo pfctl -f -
    echo "dummynet in proto tcp from any to any pipe 1" | sudo pfctl -f -

else
    sudo pfctl -F all
fi
