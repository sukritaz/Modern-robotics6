import MR6_functions as mr6
import numpy as np

# defining trajectory parameters
Tse_initial= np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.5], [0, 0, 0, 1]])
Tce_standoff = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.2], [0, 0, 0, 1]])
Tsc_initial = np.array([[1, 0, 0, 1], [0, 1, 0, 0], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tsc_final = np.array([[0, 1, 0, 0], [-1, 0, 0, -1], [0, 0, 1, 0.025], [0, 0, 0, 1]])
Tce_grasp = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1]])

# defining intial confgurations
actual_initial_config = np.array([-2, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
Tbo = np.array([[1, 0, 0, 0.1662], [0, 1, 0, 0], [0, 0, 1, 0.0026], [0, 0, 0, 1]])
Moe = np.array([[1, 0, 0, 0.033], [0, 1, 0, 0], [0, 0, 1, 0.6546], [0, 0, 0, 1]])
arm_screw_axes = np.array(
    [[0, 0, 0, 0, 0], [0, -1, -1, -1, 0], [1, 0, 0, 0, 1], [0, -0.5076, -0.3526, -0.2176, 0], [0.033, 0, 0, 0, 0],
     [0, 0, 0, 0, 0]])
# controller input
kp = 1
ki = 0.0000
dt = 0.01

true = mr6.trajectory_generator(Tse_initial, Tsc_initial, Tce_standoff, Tce_grasp, Tsc_final, "Trajectory.csv")
print(true)

mr6.produce_Tse("Trajectory.csv", "Final.csv", actual_initial_config, Tbo, Moe,
                arm_screw_axes, kp, ki, dt)
