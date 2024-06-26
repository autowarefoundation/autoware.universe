import subprocess

if __name__ == "__main__":
    params = [
        "steer_bias",
        "steer_dead_band",
        "wheel_base",
        "acc_time_delay",
        "steer_time_delay",
        "acc_time_constant",
        "steer_time_constant",
        "accel_map_scale",
        "acc_scaling",
        "steer_scaling",
        "vehicle_type",
    ]
    for param in params:
        str_run = "python3 run_sim.py " + f" --param_name {param}" + " --root auto_test"
        subprocess.run(str_run, shell=True)
