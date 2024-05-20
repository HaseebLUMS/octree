#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <port> <mode>"
    echo "mode: 'a' for average or 'i' for instantaneous"
    exit 1
fi

# Assign the provided arguments to variables
PORT=$1
MODE=$2

# Get the PID of the process using the specified port
PID=$(lsof -t -i :$PORT)

# Check if PID is not empty
if [ -z "$PID" ]; then
    echo "No process found using port $PORT."
    exit 1
fi

# Validate the mode
if [ "$MODE" != "a" ] && [ "$MODE" != "i" ] && [ "$MODE" != "ai" ]; then
    echo "Invalid mode. Please specify 'a' for average, 'i' for instantaneous or 'ai' for both"
    exit 1
fi

echo "Monitoring CPU usage for PID: $PID in $MODE mode"

# Monitor average CPU usage
while true; do
    CPU_USAGE_AVERAGE=$(ps -p $PID -o %cpu=)
    CPU_USAGE_INSTANTANEOUS=$(top -b -n 2 -d 1 -p $PID | grep $PID | tail -1 | awk '{print $9}')
    
    if [ -z "$CPU_USAGE_AVERAGE" ]; then
        echo "Process $PID not found or has terminated."
        exit 1
    fi

    if [ -z "$CPU_USAGE_INSTANTANEOUS" ]; then
        echo "Process $PID not found or has terminated."
        exit 1
    fi


    if [ "$MODE" == "a" ]; then
        echo $CPU_USAGE_AVERAGE
    fi

    if [ "$MODE" == "i" ]; then
        echo $CPU_USAGE_INSTANTANEOUS        
    fi

    if [ "$MODE" == "ai" ]; then
        echo $CPU_USAGE_AVERAGE,$CPU_USAGE_INSTANTANEOUS        
    fi

    sleep 1
done