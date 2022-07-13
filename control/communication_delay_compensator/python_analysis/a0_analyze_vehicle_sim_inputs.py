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



    ## LOAD DATA
    u_vel = load_numpy(0)
    u_steer = load_numpy(1)
    u_triag = load_numpy(3)
    u_vel_triag = load_numpy(5)
    timevec = load_numpy(2)


    # plt.figure()
    # plt.plot(timevec, u_vel)
    #
    # plt.figure()
    # plt.plot(timevec, u_steer)
    # plt.show()
    #
    # plt.figure()
    # plt.plot(timevec, u_vel_triag)
    # plt.show()

    sim_results = load_numpy(1)

    fig, ax = plt.subplots(4, 1)
    ax[0].plot(sim_results[:, 0])
    ax[0].set_title("ey")

    ax[1].plot(sim_results[:, 1])
    ax[1].set_title("epsi")

    ax[2].plot(sim_results[:, 2])
    ax[2].set_title("delta")

    ax[3].plot(sim_results[:, 3])
    ax[3].set_title("V")



    plt.show()
    a = 1

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
