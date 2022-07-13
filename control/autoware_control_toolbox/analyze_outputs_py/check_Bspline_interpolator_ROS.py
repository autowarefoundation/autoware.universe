import numpy as np
import matplotlib.pyplot as plt
import sys

sys.path.append("..")

path = "logs/bspline_interp/"

# Load the signals
sxyz_ref = np.loadtxt(path + 'sxyz_ref.txt')  # interpolated vector
sxyz_smooth = np.loadtxt(path + 'smoothed_map_ROS.txt')  # interpolated vector
tvec_base= sxyz_ref[:, 0]
tvec_smth= sxyz_smooth[:, 0]

# Curvature ROS
curvature_original = np.loadtxt(path + 'curvature_original_ROS.txt')  # interpolated vector
curvature_spline = np.loadtxt(path + 'curvature_bspline_interpolator_ROS.txt')  # interpolated vector


if __name__ == "__main__":

    plt.plot(tvec_base, sxyz_ref[:, 1], 'k-.', alpha=0.5, lw=2, label='x coord')
    plt.plot(tvec_smth, sxyz_smooth[:, 1], 'r-.', alpha=1, lw=1, label='x coord')
    plt.legend()
    plt.show()


    plt.plot(tvec_base, sxyz_ref[:, 2], 'k-.', alpha=0.5, lw=2, label='y coord')
    plt.plot(tvec_smth, sxyz_smooth[:, 2], 'r-.', alpha=1, lw=1, label='x coord')
    plt.legend()
    plt.show()

    plt.plot(tvec_base, sxyz_ref[:, 3], 'k-.', alpha=0.5, lw=2, label='z coord')
    plt.plot(tvec_smth, sxyz_smooth[:, 3], 'r-.', alpha=1, lw=1, label='x coord')
    plt.legend()
    plt.show()

    ## TEST CURVATURE
    plt.plot(tvec_base, curvature_original, 'r', lw=3, alpha=0.3, label='Original Curvature')
    plt.plot(tvec_smth, curvature_spline, 'k-.', label='BSpline Interpolator')
    plt.legend()
    plt.show()