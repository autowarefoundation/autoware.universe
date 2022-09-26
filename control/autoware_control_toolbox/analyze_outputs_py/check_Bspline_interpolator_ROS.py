import sys

import matplotlib.pyplot as plt
import numpy as np

plt.rc('figure', figsize=(12, 10))

sys.path.append("..")

path = "logs/bspline_interp/"

# Load the signals
# Down-sampled
yze = np.loadtxt(path + 'yze.txt')  # original ye
yzeinterp = np.loadtxt(path + 'yz_interp_newsize.txt')  # original ye

tbase = np.loadtxt(path + 'tvec_base.txt')  # original time coordinate
tnew = np.loadtxt(path + 'tvec_new.txt')  # downsampled time coordinate

if __name__ == "__main__":
    plt.plot(tbase, yze[:, 0], 'k-.', alpha=0.5, lw=2, label='y original')
    plt.plot(tnew, yzeinterp[:, 0], 'r-.', alpha=1, lw=1, label='y interpolated')
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(tbase, yze[:, 1], 'k-.', alpha=0.5, lw=2, label='y original')
    plt.plot(tnew, yzeinterp[:, 1], 'r-.', alpha=1, lw=1, label='y interpolated')
    plt.legend()
    plt.grid()
    plt.show()
