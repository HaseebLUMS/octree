#!/bin/bash
if [ $1 == "true" ]
then
        for PID in `ps aux | grep 'quiche_server\|quiche_client\|monitor.sh' | grep -v "grep" | awk '{print $2}'`
        do
                kill -9 ${PID}
        done
else
        for PID in `ps aux | grep 'quiche_client\|monitor.sh' | grep -v "grep" | awk '{print $2}'`
        do
                kill -9 ${PID}
        done
fi
