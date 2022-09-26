import sys

import matplotlib.pyplot as plt
import numpy as np

plt.rc('figure', figsize=(12, 10))
sys.path.append("..")

path = "logs/bspline_interp_templated/"

# Load the signals
ye = np.loadtxt(path + 'ye.txt')  # interpolated vector
yinterp = np.loadtxt(path + 'yinterp.txt')  # interpolation results

ze = np.loadtxt(path + 'ze.txt')  # interpolated vector
zinterp = np.loadtxt(path + 'zinterp.txt')  # interpolation results

# load the signal coordinates
sbase = np.loadtxt(path + 'se.txt')
snew = np.loadtxt(path + 'snew.txt')

tbase = np.linspace(0, 1, sbase.shape[0])
tnew = np.linspace(0, 1, snew.shape[0])

## Load Curvature
curvature_original = np.loadtxt(path + 'curvature_original.txt')
curvature_spline = np.loadtxt(path + 'curvature_bspline_interpolator.txt')

if __name__ == "__main__":
    plt.plot(tbase, ye, label='original data')
    plt.plot(tnew, yinterp, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.grid()
    plt.show()

    plt.plot(tbase, ze, label='original data')
    plt.plot(tnew, zinterp, label='interpolated')

    plt.legend()
    plt.grid()
    plt.show()

    ## TEST CURVATURE
    plt.plot(tbase, curvature_original, 'r', lw=3, alpha=0.3, label='Original Curvature')
    plt.plot(tnew, curvature_spline, 'k-.', label='BSpline Interpolator')
    plt.legend()
    plt.grid()
    plt.show()
    a = 1
