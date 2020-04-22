#!/bin/bash
if [ ! -d f1tenth_gym ] ; then
    git clone https://github.com/f1tenth/f1tenth_gym
else
    echo f1tenth_gym exists, not cloning.
fi
docker build -t f1tenth_gym -f Dockerfile .
