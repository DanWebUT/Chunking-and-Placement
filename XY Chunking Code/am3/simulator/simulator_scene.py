import bpy
from am3.util.simulatormath import SimulatorMath

class SimulatorScene:
    @staticmethod
    def still_running(finished_machines):
        for value in finished_machines:
            if value is False:
                return True

    @staticmethod
    def dependencies_satisfied(chunk_dependencies, finished_chunks):
        for dependency in chunk_dependencies:
            if not (dependency in finished_chunks):
                return False
        return True

    @staticmethod
    def simulate(machines):
        finished_machines = []
        finished_chunks = []
        current_chunks = []
        current_data_indices = []

        for machine in machines:
            machine.set_keyframe()
            finished_machines.append(False)
            current_chunks.append(0)
            current_data_indices.append(0)

        scene = bpy.context.scene
        bpy.context.scene.frame_set(0)

        print("SIMULATOR: Animating build")
        print("Frame estimation: {}", SimulatorMath.estimate_execution_time(machines))

        # go back to frame 0
        scene.frame_set(0)
        current_frame = 0

        for machine in machines:
            for chunk in machine.chunks:
                for (_, printed_array) in chunk.frame_data:
                    if printed_array:
                        for printed in printed_array:
                            printed.model.hide = True
                            printed.model.hide_render = True
                            printed.model.keyframe_insert("hide")
                            printed.model.keyframe_insert("hide_render")

        while SimulatorScene.still_running(finished_machines):
            if current_frame % 100 == 0:
                print("Frame {}".format(current_frame))

            scene.frame_set(current_frame - 1)
            for machine in machines:
                machine.set_last_location()
                machine.set_keyframe()
            scene.frame_set(current_frame)

            for i in range(0, len(machines)):
                if finished_machines[i] is True:
                    continue
                machine = machines[i]
                current_chunk_index = current_chunks[i]
                
                if current_chunk_index >= len(machine.chunks):
                    finished_machines[i] = True
                    continue

                current_chunk = machine.chunks[current_chunk_index]
                current_data_index = current_data_indices[i]
                chunk_frame_data = current_chunk.frame_data

                # If this chunk is ready to be printed (i.e. no filaments is needed for this region or the dependencies are satisfied)
                if current_chunk.is_empty() or SimulatorScene.dependencies_satisfied(current_chunk.dependencies, finished_chunks):
                    current_data_index += 1
                    current_data_indices[i] = current_data_index
                    if current_data_index >= len(chunk_frame_data):
                        machine.set_last_location()
                        machine.set_keyframe()

                        current_chunks[i] += 1

                        if current_chunks[i] >= len(machine.chunks):
                            finished_machines[i] = True
                        else:
                            next_chunk = machine.chunks[current_chunks[i]]
                            if next_chunk.is_empty():
                                next_location = next_chunk.frame_data[1][0]
                                machine.set_location(next_location)
                                machine.set_keyframe()
                        current_data_indices[i] = 0

                        finished_chunks.append(current_chunk.number)
                    else:
                        frame_data = chunk_frame_data[current_data_index]
                        vector = frame_data[0]
                        objects = frame_data[1]

                        machine.set_location(vector)
                        machine.set_keyframe()

                        if objects:
                            for printed in objects:
                                printed.model.hide = False
                                printed.model.hide_render = False
                                printed.model.keyframe_insert("hide")
                                printed.model.keyframe_insert("hide_render")

            current_frame += 1
            scene.frame_end = current_frame
            scene.frame_set(current_frame)

        scene.frame_end = current_frame
        print("Finished rendering")
