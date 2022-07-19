import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append("..")

path = "logs/interp_1d/"
xc = np.loadtxt(path + 'xvec.txt')  # arg for sin(x)
yc = np.loadtxt(path + 'yvec.txt')  # sin(x)

# piecewise interpolated - item by item
xnew = np.loadtxt(path + 'xnew.txt')  # arg for sin(x) downsampled
ypw = np.loadtxt(path + 'ypw.txt')  # sin(x)

# vectorwise interpolated
xnew = np.loadtxt(path + 'xnew.txt')  # arg for sin(x) downsampled
yvw = np.loadtxt(path + 'yvw.txt')  # sin(x)

if __name__ == "__main__":
    plt.plot(xc, yc, 'r', alpha=0.5, linewidth='2', label='continuous signal')
    plt.plot(xnew, ypw, 'k-.', linewidth='1', label='piecewise interpolated')
    plt.title("Sin signal to be interpolated")
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(xc, yc, 'r', alpha=0.5, linewidth='2', label='continuous signal')
    plt.plot(xnew, yvw, 'k-.', linewidth='1', label='vectorwise interpolated')
    plt.title("Sin signal to be interpolated")
    plt.legend()
    plt.grid()

    plt.show()
