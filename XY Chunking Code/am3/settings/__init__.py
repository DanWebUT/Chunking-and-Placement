MODEL_SCALE = 1
FRAMES_PER_SECOND = 4
SLICE_THICKNESS = 0.05
EXTRUSION_DIAMETER = 0.0275

def set_settings(model_scale=1, frames_per_second=30, slice_thickness=0.05, extrusion_diameter=0.0275):
    global MODEL_SCALE
    global FRAMES_PER_SECOND
    global SLICE_THICKNESS
    global EXTRUSION_DIAMETER

    MODEL_SCALE = model_scale
    FRAMES_PER_SECOND = frames_per_second
    SLICE_THICKNESS = slice_thickness
    EXTRUSION_DIAMETER = extrusion_diameter
