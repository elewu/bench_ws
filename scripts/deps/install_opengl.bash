#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

apt_install libgl1-mesa-dev
