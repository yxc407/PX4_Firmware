#! /usr/bin/env bash

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Reinstall if --reinstall set
REINSTALL_FORMULAS=""
# Install simulation tools?
INSTALL_SIM=""

# Parse arguments
for arg in "$@"
do
	if [[ $arg == "--reinstall" ]]; then
		REINSTALL_FORMULAS=$arg
	elif [[ $arg == "--sim-tools" ]]; then
		INSTALL_SIM=$arg
	fi
done

if ! command -v brew &> /dev/null
then
	# install Homebrew if not installed yet
	/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install.sh)"
fi

# Install tc-dev formula
if [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
	echo "Re-installing TC general dependencies (homebrew tc-dev)"

	# confirm Homebrew installed correctly
	brew doctor

	brew tap TC/tc
	brew reinstall tc-dev
	brew install ncurses
else
	if brew ls --versions tc-dev > /dev/null; then
		echo "tc-dev already installed"
	else
		echo "Installing TC general dependencies (homebrew tc-dev)"
		brew tap TC/tc
		brew install tc-dev
		brew install ncurses
	fi
fi

# Python dependencies
echo "Installing TC Python3 dependencies"
# We need to have future to install pymavlink later.
python3 -m pip install future
python3 -m pip install --user -r ${DIR}/requirements.txt

# Optional, but recommended additional simulation tools:
if [[ $INSTALL_SIM == "--sim-tools" ]]; then
	if brew ls --versions tc-sim > /dev/null; then
		brew install tc-sim
	elif [[ $REINSTALL_FORMULAS == "--reinstall" ]]; then
		brew reinstall tc-sim
	fi
fi

echo "All set! TC toolchain installed!"
