#!/bin/bash
#
# Script:arduino_code-uploader.sh
# Author: Divagar N
# Email: n.divagar@mobiveil.co.in
# Description: This script uploads the arduino code via cli.

if ! command -v arduino-cli &> /dev/null; then
    echo "Arduino CLI not found. Installing..."
    INSTALL_DIR="/bin"  
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh 
    if [ -f "$INSTALL_DIR/arduino-cli" ]; then
        sudo mv "$INSTALL_DIR/arduino-cli" /usr/local/bin
    else
        echo "Error: Arduino CLI binary not found in the specified installation directory."
    fi

    if ! command -v arduino-cli &> /dev/null; then
        echo "Arduino CLI installation failed. Please install it manually."
    else
        echo "Arduino CLI installed successfully."
    fi
fi

arduino-cli core install arduino:avr
arduino-cli lib install "Rosserial Arduino Library@0.7.9"


SKETCH="arduino_code-uploader/arduino_code-uploader.ino"

#git clone https://github.com/haystack-nimbus/arduino_code-uploader.git


BOARD="arduino:avr:uno"  
PORT="/dev/arduino"      

arduino-cli compile --fqbn $BOARD "$SKETCH"

arduino-cli upload -p $PORT --fqbn $BOARD "$SKETCH"

rm -rf arduino_code-uploader
