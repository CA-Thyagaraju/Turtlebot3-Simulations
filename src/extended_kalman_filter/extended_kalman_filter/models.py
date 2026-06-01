import numpy as np
import math
def motion_model(state, control, dt):

    x = state[0]
    y = state[1]
    theta = state[2]

    v = control[0]
    omega = control[1]

    x_new = (
        x +
        v * math.cos(theta) * dt
    )

    y_new = (
        y +
        v * math.sin(theta) * dt
    )

    theta_new = (
        theta +
        omega * dt
    )

    theta_new = normalize_angle(
        theta_new
    )

    return np.array([
        x_new,
        y_new,
        theta_new
    ])
def jacobian_F(state, control, dt):

    theta = state[2]

    v = control[0]

    F = np.array([

        [
            1.0,
            0.0,
            -v * math.sin(theta) * dt
        ],

        [
            0.0,
            1.0,
            v * math.cos(theta) * dt
        ],

        [
            0.0,
            0.0,
            1.0
        ]

    ])

    return F
def process_noise_Q():

    Q = np.array([

        [1e-4, 0.0, 0.0],

        [0.0, 1e-4, 0.0],

        [0.0, 0.0, 1e-3]

    ])

    return Q
def measurement_noise_R():

    R = np.array([

        [1e-3, 0.0, 0.0],

        [0.0, 1e-3, 0.0],

        [0.0, 0.0, 1e-2]

    ])

    return R

def initial_covariance_P():
    P = np.eye(3) * 0.01
    return P

def measurement_matrix_H():

    return np.eye(3)
def normalize_angle(angle):

    return math.atan2(
        math.sin(angle),
        math.cos(angle)
    )

