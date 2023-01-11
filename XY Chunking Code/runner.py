import bpy
import os
import sys

directory = os.path.join(os.path.dirname(bpy.data.filepath), 'src')
if not directory in sys.path:
    sys.path.append(directory)

import am3
from am3.panel import register, unregister

# unregister()
register()
