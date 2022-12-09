#!/bin/bash

# We need the SDK to setup an emscripten build
if [ $# -ne 1 ]; then
	echo "Usage:"
	echo "  $0 <sdk>"
	echo "Where <sdk> is the path to the emscripten SDK folder"
	exit 1
fi
SDK=$1

# Check the SDK is valid
if [ ! -f "${SDK}/emsdk_env.sh" ]; then
	echo "Incorrect SDK path"
	exit 1
fi

# Activate the SDK in the current shell
echo "Activating SDK..."
source "${SDK}/emsdk_env.sh"

# Helper to make a build folder
function make_folder {
	FOLDER=$1
	CONFIG=$2
	EMCMAKE=$3
	echo "Creating \"${CONFIG}\" build folder at \"${FOLDER}\"..."

	# Check we're not overwriting a build folder
	if [ -e "${FOLDER}" ]; then
		echo "Build folder \"${FOLDER}\" already exists - skipping"
		return
	fi

	# Make the build folder
	mkdir "${FOLDER}"
	pushd "${FOLDER}"
		${EMCMAKE} cmake ..
	popd
	
	echo "Configuration \"${CONFIG}\" created"
}

# Make the build folders
make_folder build-web Web emcmake
make_folder build-native Native
