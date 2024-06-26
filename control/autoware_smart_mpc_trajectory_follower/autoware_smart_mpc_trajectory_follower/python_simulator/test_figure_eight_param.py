import argparse
import os

from assets import ChangeParam  # type: ignore
import numpy as np
import run_sim  # type: ignore

if __name__ == "__main__":
    test_NN_size = [(16, 16), (8, 0)]
    batch_size = 5

    parser = argparse.ArgumentParser(description="Process some integers.")
    parser.add_argument("--acc_width_range", type=float)
    parser.add_argument("--acc_period_min", type=float)
    parser.add_argument("--acc_period_max", type=float)
    parser.add_argument("--steer_width_range", type=float)
    parser.add_argument("--steer_period_min", type=float)
    parser.add_argument("--steer_period_max", type=float)
    parser.add_argument("--step_response_max_input", type=float)
    parser.add_argument("--step_response_max_length", type=float)
    parser.add_argument("--step_response_interval", type=float)
    parser.add_argument("--step_response_min_length", type=float)
    parser.add_argument("--constant_vel_time", type=float)
    parser.add_argument("--use_memory_diff", action="store_true", help="Use memory diff.")
    parser.add_argument("--dir_name", type=str, default="test_vehicles")

    args = parser.parse_args()

    acc_width_range = args.acc_width_range
    acc_period_min = args.acc_period_min
    acc_period_max = args.acc_period_max
    steer_width_range = args.steer_width_range
    steer_period_min = args.steer_period_min
    steer_period_max = args.steer_period_max
    step_response_max_input = args.step_response_max_input
    step_response_max_length = args.step_response_max_length
    step_response_interval = args.step_response_interval
    step_response_min_length = args.step_response_min_length
    constant_vel_time = args.constant_vel_time
    use_memory_diff: bool = args.use_memory_diff
    dir_name = args.dir_name

    if not os.path.isdir(dir_name):
        os.mkdir(dir_name)
    condition = np.array(
        [
            acc_width_range,
            acc_period_min,
            acc_period_max,
            steer_width_range,
            steer_period_min,
            steer_period_max,
            step_response_max_input,
            step_response_max_length,
            step_response_interval,
            step_response_min_length,
            constant_vel_time,
        ]
    )
    print("condition: ", condition)
    np.savetxt(dir_name + "/figure_eight_param.csv", condition, delimiter=",")

    if use_memory_diff:
        run_sim.run_simulator(
            change_param=ChangeParam.test_vehicle,
            root=dir_name,
            batch_size=batch_size,
            hidden_layer_sizes=test_NN_size[0],
            hidden_layer_lstm=test_NN_size[1][0],
            acc_width_range=acc_width_range,
            acc_period_range=[acc_period_min, acc_period_max],
            steer_width_range=steer_width_range,
            steer_period_range=[steer_period_min, steer_period_max],
            step_response_max_input=step_response_max_input,
            step_response_max_length=step_response_max_length,
            step_response_interval=step_response_interval,
            step_response_min_length=step_response_min_length,
            constant_vel_time=constant_vel_time,
            use_memory_diff=True,
            skip_data_collection=True,
        )
    else:
        run_sim.run_simulator(
            change_param=ChangeParam.test_vehicle,
            root=dir_name,
            batch_size=batch_size,
            hidden_layer_sizes=test_NN_size[0],
            hidden_layer_lstm=test_NN_size[1][0],
            acc_width_range=acc_width_range,
            acc_period_range=[acc_period_min, acc_period_max],
            steer_width_range=steer_width_range,
            steer_period_range=[steer_period_min, steer_period_max],
            step_response_max_input=step_response_max_input,
            step_response_max_length=step_response_max_length,
            step_response_interval=step_response_interval,
            step_response_min_length=step_response_min_length,
            constant_vel_time=constant_vel_time,
            use_memory_diff=False,
        )
