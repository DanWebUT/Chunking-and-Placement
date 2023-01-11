#!/usr/bin/env bash

if [[ ! -v BLENDER_EXECUTABLE ]]; then
    BLENDER_EXECUTABLE=~/bin/blender-2.79b/blender
fi

exec ${BLENDER_EXECUTABLE} -b -P blender_chunk.py -- plane.stl 1
