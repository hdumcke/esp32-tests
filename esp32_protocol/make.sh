#!/bin/bash

mkdir -p build
g++  -Wall -std=c++17 -g esp32-proxy.cpp -o build/esp32-proxy
