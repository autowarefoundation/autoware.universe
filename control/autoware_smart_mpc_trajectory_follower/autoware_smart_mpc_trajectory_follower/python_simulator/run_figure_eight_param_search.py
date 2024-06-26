import subprocess

import numpy as np


def print_progress(msg: str):
    print("______________________________")
    print("start running:", msg)
    print("______________________________")


SEED = 44
if __name__ == "__main__":
    for k in range(100):
        np.random.seed(seed=k + SEED)
        acc_width_range = np.random.uniform(0.005, 0.1)
        acc_period_min = np.random.uniform(2.0, 10.0)
        acc_period_max = acc_period_min + np.random.uniform(5.0, 30.0)
        steer_width_range = np.random.uniform(0.005, 0.1)
        steer_period_min = np.random.uniform(2.0, 10.0)
        steer_period_max = steer_period_min + np.random.uniform(5.0, 30.0)
        step_response_max_input = np.random.uniform(0.001, 0.1)
        step_response_min_length = np.random.uniform(0.02, 1.0)
        step_response_max_length = step_response_min_length + np.random.uniform(0.03, 2.0)
        step_response_interval = step_response_max_length + np.random.uniform(1.0, 20.0)
        constant_vel_time = np.random.uniform(0.5, 20.0)

        dir_name = "test_param_search_" + str(k) + "_test_vehicle"

        for memory in range(2):
            use_memory_diff_flag = " " + ("--use_memory_diff" if memory == 1 else "")
            str_run = (
                "python3 test_figure_eight_param.py "
                + f" --acc_width_range {acc_width_range}"
                + f" --acc_period_min {acc_period_min}"
                + f" --acc_period_max {acc_period_max}"
                + f" --steer_width_range {steer_width_range}"
                + f" --steer_period_min {steer_period_min}"
                + f" --steer_period_max {steer_period_max}"
                + f" --step_response_max_input {step_response_max_input}"
                + f" --step_response_min_length {step_response_min_length}"
                + f" --step_response_max_length {step_response_max_length}"
                + f" --step_response_interval {step_response_interval}"
                + f" --constant_vel_time {constant_vel_time}"
                + f" --dir_name {dir_name}"
                + use_memory_diff_flag
            )
            print_progress(str_run)
            subprocess.run(str_run, shell=True)
