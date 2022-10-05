import robotoc_sim


class ReducedModelSimulator(robotoc_sim.LeggedSimulator):
    def __init__(self, urdf_path, time_step):
        super().__init__(urdf_path, time_step)

    @classmethod
    def get_joint_id_map(self):
        return [11, 4, 27, 28, 29, 30, 31, 32, 20, 21, 22, 23, 24, 25]