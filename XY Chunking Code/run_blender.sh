#!/usr/bin/env bash

BLENDER_EXECUTABLE=~/bin/blender-2.79b/blender

if [[ "$OSTYPE" == "darwin"* ]]; then
    BLENDER_EXECUTABLE=/Applications/Blender/blender.app/Contents/MacOS/blender
fi

# "-y" runs the bundled script automatically upon launch
exec ${BLENDER_EXECUTABLE} Simulator.blend -y
