#! /bin/bash

# Map data
if [ ! -d "data" ]; then
    mkdir data
fi
if [ ! -f "data/highway_map.csv" ]; then
    wget -P data/ https://raw.githubusercontent.com/udacity/CarND-Path-Planning-Project/master/data/highway_map.csv
fi

if [ ! -d "include" ]; then
    mkdir include
fi

# JSON
if [ ! -f "include/json.hpp" ]; then
    wget -P include/ https://github.com/nlohmann/json/releases/download/v3.7.3/json.hpp
fi

# Spline
if [ ! -f "include/spline.h" ]; then
    wget -P include/ https://kluge.in-chemnitz.de/opensource/spline/spline.h
fi

# spdlog
if [ ! -d "include/spdlog" ]; then
    wget https://github.com/gabime/spdlog/archive/v1.5.0.tar.gz
    tar -xvzf v1.5.0.tar.gz
    mv spdlog-1.5.0/include/spdlog include/
    rm -rf v1.5.0.tar.gz
fi

# Catch
if [ ! -d "lib" ]; then
    mkdir lib
fi
if [ ! -f "lib/catch.hpp" ]; then
    wget -P lib/ https://github.com/catchorg/Catch2/releases/download/v2.11.3/catch.hpp
fi
