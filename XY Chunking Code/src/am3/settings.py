"""
Contains global settings used throughout the library.
"""

MODEL_SCALE = 1.0
"""
The scaling factor for printing. Whatever the model size is, the output print size will be
``model.size * MODEL_SCALE``.

:type: float
"""

FRAMES_PER_SECOND = 4
"""
The number of frames used to represent one second of real time. Most video formats use ~30 for this
value. To simulate faster-than-real-time, use a value less than 30. For slow motion, use a value
greater than 30.

:type: float
"""

SLICE_THICKNESS = 0.5
"""
The thickness, in mm, the slicer should use when slicing models into vertical layers. The lower
this is, the finer the resolution of the print will be.

:type: float
"""

EXTRUSION_DIAMETER = 0.25
"""
The diameter of the filament, in mm, when rendering material. This is purely cosmetic and does not
impact any of the chunking, slicing, or simulating processes.

:type: float
"""

def set_settings(model_scale: float = 1,
                 frames_per_second: float = 30,
                 slice_thickness: float = 0.5,
                 extrusion_diameter: float = 0.25):
    """
    Sets the global settings to the values suppplied in the parameters to this function.

    :param model_scale: corresponds to `~am3.settings.MODEL_SCALE` (default ``1.0``)
    :type model_scale: float
    :param frames_per_second: corresponds to `~am3.settings.FRAMES_PER_SECOND` (default ``30.0``)
    :type frames_per_second: float
    :param slice_thickness: corresponds to `~am3.settings.SLICE_THICKNESS` (default ``0.5``)
    :type slice_thickness: float
    :param extrusion_diameter: corresponds to `~am3.settings.EXTRUSION_DIMATER`
        (default ``0.25``)
    :type extrusion_diameter: float
    """

    global MODEL_SCALE
    global FRAMES_PER_SECOND
    global SLICE_THICKNESS
    global EXTRUSION_DIAMETER

    MODEL_SCALE = model_scale
    FRAMES_PER_SECOND = frames_per_second
    SLICE_THICKNESS = slice_thickness
    EXTRUSION_DIAMETER = extrusion_diameter
