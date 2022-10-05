import robotoc
from robotoc_sim import MPCSimulation, CameraSettings
from fixed_upper_body_model_simulator import FixedUpperBodyModelSimulator
import numpy as np


model_info = robotoc.RobotModelInfo()
model_info.urdf_path = '../sample_robot_description/urdf/sample_robot_fixed_upper_body.urdf'
model_info.base_joint_type = robotoc.BaseJointType.FloatingBase
baumgarte_time_step = 0.05
model_info.surface_contacts = [robotoc.ContactModelInfo('L_FOOT_R', baumgarte_time_step),
                               robotoc.ContactModelInfo('R_FOOT_R', baumgarte_time_step)]
robot = robotoc.Robot(model_info)

knee_angle = np.pi / 6
step_length = np.array([0.22, 0, 0]) 
step_yaw = np.pi / 60

step_height = 0.1
swing_time = 0.7
double_support_time = 0.0
# double_support_time = 0.05
swing_start_time = 0.5

vcom_cmd = 0.5 * step_length / (swing_time+double_support_time)
yaw_rate_cmd = step_yaw / swing_time

T = 0.7
N = 25
mpc = robotoc.MPCBipedWalk(robot, T, N)

planner = robotoc.BipedWalkFootStepPlanner(robot)
planner.set_gait_pattern(step_length, step_yaw, (double_support_time > 0.))
# planner.set_raibert_gait_pattern(vcom_cmd, yaw_rate_cmd, swing_time, double_support_time, gain=0.7)
mpc.set_gait_pattern(planner, step_height, swing_time, double_support_time, swing_start_time)

# wrench cone
X = 0.08
Y = 0.04
mpc.get_contact_wrench_cone_handle().set_rectangular(X=X, Y=Y)
mpc.get_impact_wrench_cone_handle().set_rectangular(X=X, Y=Y)

t0 = 0.0
q0 = np.array([0, 0, 0, 0, 0, 0, 1,
               0, 0, -0.5*knee_angle, knee_angle, -0.5*knee_angle, 0, # left leg
               0, 0, -0.5*knee_angle, knee_angle, -0.5*knee_angle, 0]) # right leg
robot.forward_kinematics(q0)
q0[2] = - 0.5 * (robot.frame_position('L_FOOT_R')[2] + robot.frame_position('R_FOOT_R')[2]) 
v0 = np.zeros(robot.dimv())
option_init = robotoc.SolverOptions()
option_init.max_iter = 200
option_init.nthreads = 4
mpc.init(t0, q0, v0, option_init)

option_mpc = robotoc.SolverOptions()
option_mpc.max_iter = 1 # MPC iterations
option_mpc.nthreads = 4
mpc.set_solver_options(option_mpc)

time_step = 0.0025 # 400 Hz MPC
simulator = FixedUpperBodyModelSimulator(urdf_path=model_info.urdf_path, time_step=time_step)
camera_settings = CameraSettings(camera_distance=2.0, camera_yaw=45, camera_pitch=-10.0, 
                                 camera_target_pos=q0[0:3]+np.array([0.7, 1.2, 0.0]))
simulator.set_camera_settings(camera_settings=camera_settings)

simulation_time = 20.0
log = False
record = False
simulation = MPCSimulation(simulator=simulator)
simulation.run(mpc=mpc, t0=t0, q0=q0, simulation_time=simulation_time, 
               feedback_delay=True, verbose=False, 
               record=record, log=log, name='fixed_upper_body_model')

if record:
    robotoc.utils.adjust_video_duration(simulation.name+'.mp4', 
                                        desired_duration_sec=simulation_time)

if log:
    q_log = np.genfromtxt(simulation.q_log)
    v_log = np.genfromtxt(simulation.v_log)
    t_log = np.genfromtxt(simulation.t_log)
    sim_steps = t_log.shape[0]

    vcom_log = []
    wcom_log = []
    vcom_cmd_log = []
    yaw_rate_cmd_log = []
    for i in range(sim_steps):
        R = robotoc.utils.rotation_matrix_from_quaternion(q_log[i][3:7])
        robot.forward_kinematics(q_log[i], v_log[i])
        vcom_log.append(R.T@robot.com_velocity()) # robot.com_velocity() is expressed in the world coordinate
        wcom_log.append(v_log[i][3:6])
        vcom_cmd_log.append(vcom_cmd)
        yaw_rate_cmd_log.append(yaw_rate_cmd)

    plot_mpc = robotoc.utils.PlotCoMVelocity()
    plot_mpc.plot(t_log, vcom_log, wcom_log, vcom_cmd_log, yaw_rate_cmd_log, 
                  fig_name=simulation.name+'_com_vel')
