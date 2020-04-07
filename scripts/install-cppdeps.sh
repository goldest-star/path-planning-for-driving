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

# Eigen
if [ ! -d "include/Eigen" ]; then
    wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
    tar -xvzf eigen-3.3.7.tar.gz
    mv eigen-3.3.7/Eigen include/
    rm -rf eigen-3.3.7
fi

# Spline
if [ ! -f "include/spline.h" ]; then
    wget -P include/ hhttps://kluge.in-chemnitz.de/opensource/spline/spline.h
fi

# spdlog
if [ ! -d "include/spdlog" ]; then
    wget https://github.com/gabime/spdlog/archive/v1.5.0.tar.gz
    tar -xvzf v1.5.0.tar.gz
    mv spdlog-1.5.0/include/spdlog include/
    rm -rf v1.5.0.tar.gz
fi
