#!/bin/bash

set -eoux pipefail  # bash strict mode, plus print every command executed

# Base of the repo
BASEDIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && cd .. && pwd )"

# Update pip and install python modules
pip3 install -U pip
pip3 install -Ur $BASEDIR/requirements.txt
