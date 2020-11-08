import numpy as np
import csv
import modern_robotics as mr


# write required reference trajectory to csv
def format_trajectory(T, state, writer):
    for i in T:
        R = i[0:3, 0:3]
        P = i[0:3, 3]
        R = np.reshape(R, 9)
        if state:
            csv_config = np.concatenate((R, P, [1]))
            writer.writerow(csv_config)
        elif not state:
            csv_config = np.concatenate((R, P, [0]))
            writer.writerow(csv_config)


# reference trajectory from initial end effector position to cube standoff position
def Tse_initial_To_Tce_standoff(Tse_initial, Tsc_initial, Tce_standoff, status, writer):
    Tse_standoff = np.dot(Tsc_initial, Tce_standoff)
    Tse = mr.ScrewTrajectory(Tse_initial, Tse_standoff, 10, 400, 5)
    format_trajectory(Tse, status, writer)


# reference trajectory from initial cube standoff position to cube grasp position
def Tce_standoff_And_Tce_grasp(Tce_standoff, Tce_grasp, Tsc_position, status, writer):
    Tse_standoff = np.dot(Tsc_position, Tce_standoff)
    Tse_grasp = np.dot(Tsc_position, Tce_grasp)
    Tse = mr.ScrewTrajectory(Tse_standoff, Tse_grasp, 10, 200, 5)
    format_trajectory(Tse, status, writer)


# reference trajectory for gripper state
def gripper(Tce_grasp, Tsc_position, status, writer):
    Tse_grasp = np.dot(Tsc_position, Tce_grasp)
    Tse = mr.ScrewTrajectory(Tse_grasp, Tse_grasp, 10, 100, 5)
    format_trajectory(Tse, status, writer)


# reference trajectory from initial cube standoff to final cub standoff position
def Tce_standoff_To_Tce_standoff(Tsc_initial, Tce_standoff, Tsc_final, status, writer):
    Tse_standoff_initial = np.dot(Tsc_initial, Tce_standoff)
    Tse_standoff_final = np.dot(Tsc_final, Tce_standoff)
    Tse = mr.ScrewTrajectory(Tse_standoff_initial, Tse_standoff_final, 10, 400, 5)
    format_trajectory(Tse, status, writer)


# produce the 8 segment trajectory
def trajectory_generator(Tse_initial, Tsc_initial, Tce_standoff, Tce_grasp, Tsc_final, csv_file):
    f = open(csv_file, "w")
    writer = csv.writer(f)
    # 1: A trajectory to move the gripper from its initial configuration to a "standoff" configuration a few cm above
    # the block
    Tse_initial_To_Tce_standoff(Tse_initial, Tsc_initial, Tce_standoff, False, writer)
    # 2:A trajectory to move the gripper down to the grasp position
    Tce_standoff_And_Tce_grasp(Tce_standoff, Tce_grasp, Tsc_initial, False, writer)
    # 3: Closing of the gripper.
    gripper(Tce_grasp, Tsc_initial, True, writer)
    # 4: A trajectory to move the gripper back up to the "standoff" configuration.
    Tce_standoff_And_Tce_grasp(Tce_grasp, Tce_standoff, Tsc_initial, True, writer)
    # 5: A trajectory to move the gripper to a "standoff" configuration above the final configuration.
    Tce_standoff_To_Tce_standoff(Tsc_initial, Tce_standoff, Tsc_final, True, writer)
    # 6: A trajectory to move the gripper to the final configuration of the object.
    Tce_standoff_And_Tce_grasp(Tce_standoff, Tce_grasp, Tsc_final, True, writer)
    # 7: Opening of the gripper.
    gripper(Tce_grasp, Tsc_final, False, writer)
    # 8: A trajectory to move the gripper back to the "standoff" configuration.
    Tce_standoff_And_Tce_grasp(Tce_grasp, Tce_standoff, Tsc_final, False, writer)
    print("Trajectory generated")
    f.close()
    return 0


# convert to regular dictionary
def ordereddict_to_dict(value):
    for k, v in value.items():
        if isinstance(v, dict):
            value[k] = ordereddict_to_dict(v)
    return dict(value)


# convert csv row to SE3
def Transform_To_SE3_from_csv(row):
    row = ordereddict_to_dict(row)
    for k, v in row.items():
        row[k] = float(v)
    Tse = np.array([[row["r11"], row["r12"], row["r13"], row["px"]], [row["r21"], row["r22"], row["r23"], row["py"]],
                    [row["r31"], row["r32"], row["r33"], row["pz"]], [0, 0, 0, 1]])
    return Tse


# convert joint configuration to SE3
def config_To_SE3(theta_list, Tbo, Moe, blist):
    chassis_config = theta_list[0:3]
    Tsb = np.array([[np.cos(chassis_config[0]), -np.sin(chassis_config[0]), 0, chassis_config[1]],
                    [np.sin(chassis_config[0]), np.cos(chassis_config[0]), 0, chassis_config[2]], [0, 0, 1, 0.0963],
                    [0, 0, 0, 1]])
    arm_config = theta_list[3:8]
    Toe = mr.FKinBody(Moe, blist, arm_config)
    Tse = np.dot(np.dot(Tsb, Tbo), Toe)
    return Tse


# calculate Je
def calc_je(_Moe, _blist, arm_config, _Tbo):
    l = 0.235
    w = 0.15
    r = 0.0475
    H = np.array([[-l - w, 1, -1], [l + w, 1, 1], [l + w, 1, -1], [-l - w, 1, 1]])
    F = np.array(r * np.linalg.pinv(H))
    F = np.concatenate((np.zeros((2, F.shape[1])), F, np.zeros((1, F.shape[1]))), axis=0)
    Toe = mr.FKinBody(_Moe, _blist, arm_config)
    J_base = np.dot(mr.Adjoint(np.dot(np.linalg.inv(Toe), np.linalg.inv(_Tbo))), F)  # J base successfully calculated
    J_arm = mr.JacobianBody(_blist, arm_config)
    Je = np.concatenate((J_base, J_arm), axis=1)
    return Je


# calculate required joint velocities
def calculate_joint_velocity(Moe, arm_screw_axes, theta_list, Tbo, v_e):
    J = calc_je(Moe, arm_screw_axes, theta_list[3:8], Tbo)
    return np.dot(np.linalg.pinv(J), v_e)


# function to evaluate chassis twist vb in {b}
def calc_vb(wheel_speed):
    l = 0.235
    w = 0.15
    r = 0.0475
    H = np.array([[-l - w, 1, -1], [l + w, 1, 1], [l + w, 1, -1], [-l - w, 1, 1]])
    F = r * np.linalg.pinv(H)
    return np.dot(F, wheel_speed.T)


# function to evaluate delta position in {b}
def calc_delta_qb(vb):
    if vb[0] == 0:
        delta_q = vb
    else:
        delta_q = [vb[0], (vb[1] * np.sin(vb[0]) + vb[2] * (np.cos(vb[0]) - 1)) / vb[0],
                   (vb[2] * np.sin(vb[0]) + vb[1] * (1 - np.cos(vb[0]))) / vb[0]]
    return delta_q


# function to evaluate delta position in {s}
def calc_delta_qs(delta_qb, theta):
    transformation_matrix = np.array([[1, 0, 0], [0, np.cos(theta), -np.sin(theta)], [0, np.sin(theta), np.cos(theta)]])
    return np.dot(transformation_matrix, delta_qb)


# function to get next configuration in SE3 and write it to csv
def nextstate(config, timestep, joint_speed, writer, state):
    arm_joint_speed = joint_speed[4:9]
    wheel_speed = joint_speed[0:4]
    chassis_config = config[0:3]
    arm_config = config[3:8]
    wheel_angle = config[8:12]
    # odometry
    vb = calc_vb(wheel_speed)
    delta_qb = calc_delta_qb(vb)
    delta_qs = calc_delta_qs(delta_qb, chassis_config[0])
    # evaluation of next position values
    next_chassis_config = chassis_config + delta_qs * timestep
    next_arm_config = arm_config + arm_joint_speed * timestep
    next_wheel_angle = wheel_angle + wheel_speed * timestep
    # writing to csv file
    csv_config = np.concatenate((next_chassis_config, next_arm_config, next_wheel_angle, state))
    next_state = csv_config
    writer.writerow(csv_config)
    return next_state


# get complete 8segment trajectory and define feedforward + PI controller
def produce_Tse(trajectory_file, Final_config_file, actual_initial_config, Tbo, Moe,
                arm_screw_axes, kp, ki, dt):
    integral_error = 0
    theta_list = actual_initial_config
    l = open("log error.csv", "w")
    writer1 = csv.writer(l)
    f = open(trajectory_file, "r")
    c = open(Final_config_file, "w")
    writer = csv.writer(c)
    reader = csv.DictReader(f,
                            ("r11", "r12", "r13", "r21", "r22", "r23", "r31", "r32", "r33", "px", "py", "pz", "state"))
    count = -1
    Tse = config_To_SE3(theta_list, Tbo, Moe, arm_screw_axes)  # initializing, Tse will get updated
    for row in reader:
        count += 1
        if count == 0:
            xd = row
        else:
            xd_next = row
            Ts_d = Transform_To_SE3_from_csv(xd)
            Ts_dnext = Transform_To_SE3_from_csv(xd_next)
            xd = xd_next
            v_sd = mr.se3ToVec((1 / dt) * (
                mr.MatrixLog6(np.dot(np.linalg.inv(Ts_d), Ts_dnext))))  # desired twist in s frame {s}
            v_ed = np.dot(mr.Adjoint(np.dot(np.linalg.inv(Tse), Ts_d)), v_sd)  # desired twist in end-effector frame {e}
            x_err = mr.se3ToVec(mr.MatrixLog6(np.dot(np.linalg.inv(Tse), Ts_d)))
            writer1.writerow(x_err)
            integral_error = integral_error + x_err * dt
            v_e = v_ed + kp * x_err + ki * integral_error
            u = calculate_joint_velocity(Moe, arm_screw_axes, theta_list, Tbo, v_e)
            theta_list = nextstate(theta_list, dt, u, writer, [float(row["state"])])
            Tse = config_To_SE3(theta_list, Tbo, Moe, arm_screw_axes)
    print("x_err logged")
    f.close()
    l. close()
    c.close()
