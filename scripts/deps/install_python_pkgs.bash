set -e
BASEDIR=$(dirname "$0")
source "$BASEDIR/config.bash"

# Python header files and library
apt_install python-dev
apt_install python3-dev

# Pip
apt_install python-pip
apt_install python3-pip

# Matplotlib
apt_install python-matplotlib
apt_install python-matplotlib-data
apt_install python-matplotlib-dbg
apt_install python-matplotlib-doc
apt_install python3-matplotlib
apt_install python3-matplotlib-dbg

# Both required by rpy_trajectory_evaluate
sudo -H -u ${SUDO_USER} bash -c 'pip install -U PyYAML'
sudo -H -u ${SUDO_USER} bash -c 'pip install -U colorama'
