import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append("..")

path = "logs/helper_funcs/"
xc = np.loadtxt(path + 'xc.txt')  # a continous angle series signal
xw = np.loadtxt(path + 'xw.txt')  # a wrppaed angle series signal
uw_x = np.loadtxt(path + 'uw_x.txt')  # a continous angle series signal

if __name__ == "__main__":

    fig, axs = plt.subplots(1, 2)
    axs[0].plot(xc, 'k', alpha=0.2, linewidth='3', label='continuous signal')
    axs[0].plot(uw_x, 'r-.', alpha=0.8, label='unwrapped matlab')
    axs[0].set_title("Continuous signal")

    axs[1].plot(xw, label='wrapped signal')
    axs[1].set_title("Wrapped signal signal")

    for ax in axs.reshape(-1):
        ax.grid()
        ax.legend()

    plt.show()
