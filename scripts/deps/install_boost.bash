#!/bin/bash
set -e  # exit on first error
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

apt_install libboost-dev
apt_install libboost-signals-dev
