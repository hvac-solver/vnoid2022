import robotoc
from robotoc_sim import MPCSimulation, CameraSettings
from reduced_model_simulator import ReducedModelSimulator
import numpy as np


model_info = robotoc.RobotModelInfo()
model_info.urdf_path = '../model/sample_robot_description/urdf/sample_robot_reduced.urdf'
model_info.base_joint_type = robotoc.BaseJointType.FloatingBase
baumgarte_time_step = 0.05
model_info.surface_contacts = [robotoc.ContactModelInfo('L_FOOT_R', baumgarte_time_step),
                               robotoc.ContactModelInfo('R_FOOT_R', baumgarte_time_step)]
robot = robotoc.Robot(model_info)

print(robot)

knee_angle = np.pi / 6

q0 = np.array([0, 0, 0, 0, 0, 0, 1,
               0, # left sholder
               0, # right sholder
               0, 0, -0.5*knee_angle, knee_angle, -0.5*knee_angle, 0, # left leg
               0, 0, -0.5*knee_angle, knee_angle, -0.5*knee_angle, 0]) # right leg
robot.forward_kinematics(q0)
q0[2] = - 0.5 * (robot.frame_position('L_FOOT_R')[2] + robot.frame_position('R_FOOT_R')[2]) 

print(q0)