#!/bin/bash

# Check if port number is provided as an argument
if [ -z "$1" ]; then
    echo "Usage: $0 <port>"
    exit 1
fi

# Assign the provided argument to the PORT variable
PORT=$1

# Get the PID of the process using the specified port
PID=$(lsof -t -i :$PORT)

# Check if PID is not empty
if [ -z "$PID" ]; then
    echo "No process found using port $PORT."
    exit 1
fi

echo "Monitoring CPU usage for PID: $PID"

# Monitor CPU usage of the process every second
while true; do
    # Get CPU usage using top
    CPU_USAGE=$(top -b -n 2 -d 1 -p $PID | grep $PID | tail -1 | awk '{print $9}')
    
    # Check if CPU_USAGE is not empty (process may have terminated)
    if [ -z "$CPU_USAGE" ]; then
        echo "Process $PID not found or has terminated."
        exit 1
    fi
    
    # Print the CPU usage
    echo $CPU_USAGE
    
    # Wait for 1 second
    sleep 1
done
