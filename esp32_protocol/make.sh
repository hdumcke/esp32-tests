#!/bin/bash

mkdir -p build
g++ -o build/uart.o uart.c -c -Wall -std=c++17 -g
g++  -Wall -std=c++17 -g esp32-server.cpp -o build/esp32-server build/uart.o
