import numpy as np
from f1tenth_pysim.dynamic_models_ros import vehicle_dynamics_st_update_pacejka

import time


class single_track_model:
    """
    Single Track Model for ukf_node. Uses model defined in the vehicle_dynamics_st_update_pacejka function of f1tenth_pysim.dynamic_models_ros

        State x = [x, y, steer_angle, velocity, theta, angular_velocity, slip_angle]
        Control Input u = [accel, steer]

        Threshold value for mixing logic v_b is tuned empirically to 2.1 (value is defined in vehicle_dynamics_st_update_pacejka)
    """

    def __init__(self):
        self.dim_x = 7

        self.dim_u = 2
        self.u = np.zeros(self.dim_u)
        self.u_index = np.array([13, 12])

        self.x = np.zeros(self.dim_x)
        self.P = np.eye(self.dim_x) * 100

        # Measure Computation time of one system function call
        self.max_time = 0
        self.count = 0
        self.mean_time = 0

    ################
    # System Model #
    ################

    def fx(self, x, dt, u, p):
        accel = u[0]
        steer = u[1]
        new_state, dots = vehicle_dynamics_st_update_pacejka(x, accel, steer, p, dt)
        return new_state

    #####################
    # Measurement Model #
    #####################

    def h_vesc(self, x, **kwargs):
        # vesc gives [x, y, psi, vx, vy, vpsi]
        vx = x[3] * np.cos(x[6])
        vy = x[3] * np.sin(x[6])
        return np.array([x[0], x[1], x[4], vx, vy, x[5]]).flatten()

    def h_imu(self, x, u, p, **kwargs):
        # z_imu = [psi, vpsi, ax, ay]
        t0 = time.perf_counter_ns()
        _, dots = vehicle_dynamics_st_update_pacejka(x, u[0], u[1], p, dt=0.0)  # dt=0 just for dots
        t1 = time.perf_counter_ns()
        duration_ms = (t1 - t0) / 1_000_000.0
        if duration_ms > self.max_time:
            self.max_time = duration_ms

        self.count += 1
        self.mean_time += (duration_ms - self.mean_time) / self.count

        # Combine dots with rotational effects
        imu_ax = dots[0]
        imu_ay = dots[1]

        return np.array([x[4], x[5], imu_ax, imu_ay]).flatten()

    # Custom Residuals
    def normalize_angle(self, x):
        return np.arctan2(np.sin(x), np.cos(x))

    def residual_x(self, a, b):
        y = a - b
        y[2] = self.normalize_angle(y[2])
        y[4] = self.normalize_angle(y[4])
        y[6] = self.normalize_angle(y[6])
        return y

    def residual_imu(self, a, b):
        y = a - b
        y[0] = self.normalize_angle(y[0])
        return y

    def residual_vesc(self, a, b):
        y = a - b
        y[2] = self.normalize_angle(y[2])
        return y

    # Custom mean functions
    def state_mean(self, sigmas, Wm):
        x = np.zeros(7)

        x[0] = np.sum(np.dot(sigmas[:, 0], Wm))
        x[1] = np.sum(np.dot(sigmas[:, 1], Wm))

        # circular mean for steer_angle
        x[2] = np.arctan2(
            np.sum(np.dot(np.sin(sigmas[:, 2]), Wm)),
            np.sum(np.dot(np.cos(sigmas[:, 2]), Wm)))

        x[3] = np.sum(np.dot(sigmas[:, 3], Wm))

        # circular mean for psi
        x[4] = np.arctan2(
            np.sum(np.dot(np.sin(sigmas[:, 4]), Wm)),
            np.sum(np.dot(np.cos(sigmas[:, 4]), Wm)))

        x[5] = np.sum(np.dot(sigmas[:, 5], Wm))

        # circular mean for slip
        x[6] = np.arctan2(
            np.sum(np.dot(np.sin(sigmas[:, 6]), Wm)),
            np.sum(np.dot(np.cos(sigmas[:, 6]), Wm)))

        return x

    def imu_mean(self, sigmas, Wm):
        z = np.zeros(4)

        sum_sin = np.sum(np.dot(np.sin(sigmas[:, 0]), Wm))
        sum_cos = np.sum(np.dot(np.cos(sigmas[:, 0]), Wm))
        z[0] = np.arctan2(sum_sin, sum_cos)

        z[1] = np.sum(np.dot(sigmas[:, 1], Wm))
        z[2] = np.sum(np.dot(sigmas[:, 2], Wm))
        z[3] = np.sum(np.dot(sigmas[:, 3], Wm))

        return z

    def vesc_mean(self, sigmas, Wm):
        z = np.zeros(6)
        z[0] = np.sum(np.dot(sigmas[:, 0], Wm))  # x
        z[1] = np.sum(np.dot(sigmas[:, 1], Wm))  # y

        # circular mean for psi
        z[2] = np.arctan2(
            np.sum(np.dot(np.sin(sigmas[:, 2]), Wm)),
            np.sum(np.dot(np.cos(sigmas[:, 2]), Wm)))

        z[3] = np.sum(np.dot(sigmas[:, 3], Wm))  # vx
        z[4] = np.sum(np.dot(sigmas[:, 4], Wm))  # vy
        z[5] = np.sum(np.dot(sigmas[:, 5], Wm))  # vpsi

        return z

    # Helper Functions
    def get_odom_state(self, state):
        """
        Convert state of model to right format for odom message
        """

        vx = state[3] * np.cos(state[6])
        vy = state[3] * np.sin(state[6])

        odom_state = np.array([
            state[0],
            state[1],
            state[4],
            vx,
            vy,
            state[5]
        ])

        return odom_state
