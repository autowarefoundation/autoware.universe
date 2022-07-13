import os
import matplotlib.pyplot as plt
import numpy as np

data_path = "/mnt/KinsgtonExternal/ROS2_universe/logs"
# Press the green button in the gutter to run the script.

files = sorted(os.listdir(data_path))
file_list = []
file_names = []

for f in files:
    if f.endswith(".txt"):
        print(f)
        file_list.append(data_path + f)
        file_names.append(f)

    file_list.sort()
    file_names.sort()

def load_numpy(k):
    f0 = data_path + "/" + file_names[k]

    return np.loadtxt(f0)

if __name__ == '__main__':

    for (f, k) in zip(file_names, range(len(file_names))):
        print(f" k: {k}, file_name: {f}")


    sim_results_nodelay = np.loadtxt(data_path + "/" + "sim_results_nodelay.txt")
    sim_results_with_delay = np.loadtxt(data_path + "/" + "sim_results_with_delay.txt")


    fig, ax = plt.subplots(4, 1)
    ax[0].plot(sim_results_nodelay[:, 0], label ='no delay')
    ax[0].plot(sim_results_with_delay[:, 0])
    ax[0].set_title("ey")

    ax[1].plot(sim_results_nodelay[:, 1], label ='no delay')
    ax[1].plot(sim_results_with_delay[:, 1])
    ax[1].set_title("epsi")

    ax[2].plot(sim_results_nodelay[:, 2], label ='no delay')
    ax[2].plot(sim_results_with_delay[:, 2])
    ax[2].set_title("delta")

    ax[3].plot(sim_results_nodelay[:, 3], label ='no delay')
    ax[3].plot(sim_results_with_delay[:, 3])
    ax[3].set_title("V")

    plt.tight_layout()
    plt.legend()
    plt.show()
    a = 1

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
