import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append("..")

path = "../analyze_outputs_py/logs/spline_pcg/"

se = np.loadtxt(path + 'se.txt')  # interpolating coordinates  s = sum(ds)
ye = np.loadtxt(path + 'ye.txt')  # base vector - sin(x)
ze = np.loadtxt(path + 'ze.txt')  # base vector - cos(x)

snew = np.loadtxt(path + 'snew.txt')  # new coordinates
yinterp = np.loadtxt(path + 'yinterp.txt')  # interpolation results
zinterp = np.loadtxt(path + 'zinterp.txt')  # interpolation results

snew2 = np.loadtxt(path + 'se_new2.txt')  # new coordinates
zinterp2 = np.loadtxt(path + 'zinterp2.txt')  # interpolation results

# Itemwise
zitemwise = np.loadtxt(path + 'zinterp_itemwise.txt')  # interpolation results

# vectorwise
snew3 = np.loadtxt(path + 'snew3.txt')  # new coordinates
zvectorwise = np.loadtxt(path + 'zvectorwise.txt')  # interpolation results

# Linear Interpolation
ylinear = np.loadtxt(path + 'ylinear.txt')  # interpolation results

if __name__ == "__main__":
    plt.plot(se, ye, 'r', alpha=0.5, lw=1, label='original data y = sin(x)')
    plt.plot(snew, yinterp, 'k-.', label='interpolated y = sin(x)')
    plt.legend()
    plt.title('s arclength z = sin(x)')
    plt.show()

    plt.plot(se, ze, 'r', alpha=0.5, lw=1, label='original data z = cos(x)')
    plt.plot(snew, zinterp, 'k-.', label='interpolated')
    # plt.plot(snew, zinterp, 'go', alpha=0.5, label='interpolated')

    plt.title('s arclength z = cos(x)')
    plt.legend()
    plt.show()

    plt.plot(se, ze, 'r', alpha=0.5, lw=1, label='original data z = cos(x)')
    plt.plot(snew2, zinterp2, 'k-.', label='interpolated with different steps')
    # plt.plot(snew, zinterp, 'go', alpha=0.5, label='interpolated')

    plt.title('s arclength z = cos(x)')
    plt.legend()
    plt.show()

    plt.plot(se, ze, 'r', alpha=0.5, lw=1, label='original data z = cos(x)')
    plt.plot(snew2, zitemwise, 'k-.', label='interpolated with itemwise')
    # plt.plot(snew, zinterp, 'go', alpha=0.5, label='interpolated')

    plt.title('s arclength z = cos(x)')
    plt.legend()
    plt.show()

    plt.plot(se, ze, 'r', alpha=0.5, lw=1, label='original data z = cos(x)')
    plt.plot(snew3, zvectorwise, 'k-.', label='interpolated with vectorwise')
    # plt.plot(snew, zinterp, 'go', alpha=0.5, label='interpolated')

    plt.title('s arclength z = cos(x)')
    plt.legend()
    plt.show()

    plt.plot(se, ye, 'r', alpha=0.5, lw=1, label='original data y = sin(x)')
    plt.plot(snew, ylinear, 'k-.', label='linear interpolated y = sin(x)')
    plt.legend()
    plt.title('s arclength z = sin(x)')
    plt.show()
