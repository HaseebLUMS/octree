#!/bin/bash

export CXX=/usr/bin/clang++

GREEN='\033[0;32m'
COLOR_END='\033[0m'

echo "Setting OS UDP Buffers"

sudo sysctl -w net.core.rmem_default=26214400
sudo sysctl -w net.core.wmem_default=26214400

sudo sysctl -w net.core.wmem_max=26214400
sudo sysctl -w net.core.rmem_max=26214400

sudo apt-get install -y libev-dev uthash-dev libavif-dev libpcl-dev libturbojpeg0-dev libturbojpeg

# echo -e "${GREEN}Adding FQ to lo${COLOR_END}"

# sudo tc qdisc add dev lo root fq maxrate 2gbit

echo "Running submodule update"
git submodule update --init --recursive

mkdir build
mkdir third_party_build

echo "Use cmake -S . -B build/"