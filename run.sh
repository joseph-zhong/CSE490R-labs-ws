#!/bin/bash

echo $1
echo $2

cmd="roslaunch lab1 ParticleFilter.launch \
  max_particles:=$1 \
  resample_type:=$2"

echo "Running $cmd"
$cmd
