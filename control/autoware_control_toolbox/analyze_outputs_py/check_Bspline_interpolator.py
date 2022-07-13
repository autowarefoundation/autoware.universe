import numpy as np
import matplotlib.pyplot as plt
import sys

sys.path.append("..")

path = "logs/bspline_interp/"

# Load the signals
yze = np.loadtxt(path + 'yze.txt')  # interpolated vector
yz_interp = np.loadtxt(path + 'yz_interp_newsize.txt')  # interpolation results

# load the signal coordinates
tvec_new = np.loadtxt(path + 'tvec_new.txt')
tvec_base = np.loadtxt(path + 'tvec_base.txt')

## Check the results when the new coordinate vectors are given.
tvec_new_1 = np.loadtxt(path + 'tvec_new_1.txt')
tvec_base_1 = np.loadtxt(path + 'tvec_base_1.txt')
yz_interp_new_coord = np.loadtxt(path + 'yz_interp_new_coord.txt')  # interpolation results

## Load Curvature
curvature_original = np.loadtxt(path + 'curvature_original.txt')
curvature_spline = np.loadtxt(path + 'curvature_original.txt')

if __name__ == "__main__":
    plt.plot(tvec_base, yze[:, 0], 'k-.', alpha=0.5, lw=2, label=' y data')

    # plt.plot(xe, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(tvec_new, yz_interp[:, 0], 'r', lw=3, alpha=0.3, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    plt.plot(tvec_base, yze[:, 1], 'k-.', alpha=0.5, lw=2, label=' y data')

    # plt.plot(xe, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(tvec_new, yz_interp[:, 1], 'r', lw=3, alpha=0.3, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    ## ----------  Test the case when we construct with the coordinate vectors. ----------
    plt.plot(tvec_base_1, yze[:, 0], 'k-.', alpha=0.5, lw=2, label=' y data')

    # plt.plot(xe, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(tvec_new_1, yz_interp_new_coord[:, 0], 'r', lw=3, alpha=0.3, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    plt.plot(tvec_base_1, yze[:, 1], 'k-.', alpha=0.5, lw=2, label=' y data')

    # plt.plot(xe, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(tvec_new_1, yz_interp_new_coord[:, 1], 'r', lw=3, alpha=0.3, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    ## TEST CURVATURE
    plt.plot(tvec_base_1, curvature_original, 'r', lw=3, alpha=0.3, label='Original Curvature')
    plt.plot(tvec_base_1, curvature_spline, 'k-.', label='BSpline Interpolator')
    plt.legend()
    plt.show()
    a = 1
