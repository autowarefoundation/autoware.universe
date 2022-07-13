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

    q_simresults_fromABCD = np.loadtxt(data_path + "/" + "q_simresults_fromABCD.txt")
    q_simresults_fromACT = np.loadtxt(data_path + "/" + "q_simresults_fromACT.txt")

    fig, ax = plt.subplots(3, 1)
    ax[0].plot(q_simresults_fromABCD[:, 0], label=' from ABCD')
    ax[0].plot(q_simresults_fromACT[:, 0], label=' from SS in ACT')
    ax[0].set_title("Input to the system")

    ax[1].plot(q_simresults_fromABCD[:, 1], label='from ABCD')
    ax[1].plot(q_simresults_fromACT[:, 1], label='from SS in ACT')
    ax[1].set_title("Filtered y")

    ax[2].plot(q_simresults_fromABCD[:, 2], label='from ABCD')
    ax[2].plot(q_simresults_fromACT[:, 2], label='from SS in ACT')
    ax[2].set_title("Filtered epsi")

    plt.tight_layout()
    plt.legend()
    plt.show()
    a = 1

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
