cmake_minimum_required(VERSION 3.31)
project(pololu_test C)

set(CMAKE_C_STANDARD 11)

add_executable(pololu_test main.c
        pololu_i2c.h
        pololu_i2c.c
        LICENSE)
