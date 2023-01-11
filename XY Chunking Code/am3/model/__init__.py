from mathutils import Vector

class Chunk:
    def __init__(self, model, number = -1):
        self.model = model
        self.row = 0
        self.number = number
        self.dependencies = []
        self.slices = []
        self.commands = []
        self.color = (255, 255, 255)
        self.execution_time = -1
        self.frame_data = [] # [(Vector, [3D Objects])]

    def __getitem__(self, index):
        return self.slices[index]

    def __setitem__(self, index, item):
        self.slices[index] = item

    def __iter__(self):
        return self.slices.__iter__()

    def append(self, item):
        if isinstance(item, list):
            for sub_item in item:
                self.slices.append(sub_item)
        else:
            self.slices.append(item)

    def set_name(self, name):
        if not self.is_empty():
            self.model.name = name

    def set_hidden(self, hidden):
        self.model.hide = hidden

    def add_dependency(self, number):
        if isinstance(number, list):
            for value in number:
                self.dependencies.append(value)
        else:
            self.dependencies.append(number)

    def is_empty(self):
        return isinstance(self.model, Vector)

    def get_min_position(self):
        min_z = 0
        for slice in self.slices:
            for path in slice.paths:
                for vector in path.vectors:
                    if vector[2] < min_z:
                        min_z = vector[2]
        return min_z

    def shift_paths(self, z_position):
        for slice in self.slices:
            for path in slice.paths:
                for i in range(0, len(path.vectors)):
                    vector = path.vectors[i]
                    x = vector[0]
                    y = vector[1]
                    z = vector[2] + z_position
                    new_vector = Vector((x, y, z))
                    path.vectors[i] = new_vector




class Slice:
    def __init__(self):
        self.paths = []

    def __getitem__(self, index):
        return self.paths[index]

    def __setitem__(self, index, item):
        self.paths[index] = item

    def __iter__(self):
        return self.paths.__iter__()

    def append(self, item):
        if isinstance(item, list):
            for sub_item in item:
                self.paths.append(sub_item)
        else:
            self.paths.append(item)


class Path:
    def __init__(self):
        self.vectors = []

    def __getitem__(self, index):
        return self.vectors[index]

    def __setitem__(self, index, item):
        self.vectors[index] = item

    def __iter__(self):
        return self.vectors.__iter__()

    def append(self, item):
        if isinstance(item, list):
            for sub_item in item:
                self.vectors.append(sub_item)
        else:
            self.vectors.append(item)
