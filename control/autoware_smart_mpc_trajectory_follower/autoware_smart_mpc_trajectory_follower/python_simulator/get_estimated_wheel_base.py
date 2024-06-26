from autoware_smart_mpc_trajectory_follower.scripts import drive_functions
import numpy as np


def get_estimated_wheel_base_coef(dir_name: str) -> float:
    A = np.load(dir_name + "/polynomial_reg_info.npz")["A"]
    estimated_wheel_base = 1 / (1 / drive_functions.L + A[3, 11])
    print("estimated wheel base:", estimated_wheel_base)
    return estimated_wheel_base / drive_functions.L
