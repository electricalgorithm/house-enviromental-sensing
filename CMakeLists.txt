# Set the board to use for this project.
set(BOARD esp32)

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Set the project name.
project(HouseEnvSensBLE)

target_sources(app PRIVATE src/main.c)
# zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
