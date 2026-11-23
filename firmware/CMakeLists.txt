# USV GPS/RTK module simulator for ESP32-C3 — ESP-IDF project.
cmake_minimum_required(VERSION 3.16)

# Pull in the ESP-IDF project machinery. IDF_PATH is set by buildenv.ps1.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(usv_gps_sim)
