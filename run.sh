#!/bin/bash

echo $1
echo $2

cmd="roslaunch lab2 Controller.launch \
  controller_type:=$1"

echo "Running $cmd"
$cmd
