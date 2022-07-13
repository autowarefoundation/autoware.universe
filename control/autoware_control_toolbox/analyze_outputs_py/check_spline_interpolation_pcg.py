import numpy as np
import matplotlib.pyplot as plt
import sys

sys.path.append("..")

path = "logs/spline_pcg/"

se = np.loadtxt(path + 'se.txt')  # interpolating coordinates
ye = np.loadtxt(path + 'ye.txt')  # interpolated vector
ze = np.loadtxt(path + 'ze.txt')  # interpolated vector

snew = np.loadtxt(path + 'snew.txt')  # new coordinates
yinterp = np.loadtxt(path + 'yinterp.txt')  # interpolation results
zinterp = np.loadtxt(path + 'zinterp.txt')  # interpolation results

snew2 = np.loadtxt(path + 'se_new2.txt')  # new coordinates
zinterp2 = np.loadtxt(path + 'zinterp2.txt')  # interpolation results

if __name__ == "__main__":
    plt.plot(se, ye, 'r', alpha=0.5, lw=4, label='original data')
    # plt.plot(se, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(snew, yinterp, label='interpolated')
    # plt.plot(snew, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    plt.plot(se, ze, 'r', alpha=0.5, lw=4, label='original data')
    plt.plot(se, ze, 'rs', alpha=0.5, label='original data')
    plt.plot(snew, zinterp, label='interpolated')
    plt.plot(snew, zinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    plt.plot(se, ze, 'r', alpha=0.5, lw=4, label='original data')
    plt.plot(snew2, zinterp2, label='interpolated')

    plt.legend()
    plt.show()
