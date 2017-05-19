#!/bin/bash

# Build project
# Number of jobs = 2 * cores + 1 due to Hyper-Threading
cmake --build build -- -j 13 #-- -j $(($(nproc)+1)) #

