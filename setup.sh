#!/bin/bash

export CXX=/usr/bin/clang++

echo "Setting OS UDP Buffers"
sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.rmem_max=26214400