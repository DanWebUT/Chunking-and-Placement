import bpy
import os
import sys

directory = os.path.dirname(bpy.data.filepath)
if not directory in sys.path:
    sys.path.append(directory)

import am3
import am3.panel
import am3.panel.panel
from am3.panel import *
from am3.panel.panel import SimulatorPanel, FileProperty, ChunkingOperator, register, unregister

# unregister()
register()
