import os
from pathlib import Path
import shutil

import smart_mpc_trajectory_follower

if __name__ == "__main__":
    package_dir = str(Path(smart_mpc_trajectory_follower.__file__).parent)

    remove_dirs = [
        package_dir + "/__pycache__",
        package_dir + "/scripts/__pycache__",
        package_dir + "/training_and_data_check/__pycache__",
    ]
    for i in range(len(remove_dirs)):
        if os.path.isdir(remove_dirs[i]):
            shutil.rmtree(remove_dirs[i])
