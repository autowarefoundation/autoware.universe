import numpy as np
import matplotlib.pyplot as plt
import sys

sys.path.append("..")

path = "logs/bspline_smooth/"

se = np.loadtxt(path + 'se.txt')  # interpolating coordinates
xe = np.loadtxt(path + 'xe.txt')  # interpolated vector
ye = np.loadtxt(path + 'ye.txt')  # interpolated vector
yze = np.loadtxt(path + 'yze.txt')  # interpolated vector

xvec = np.loadtxt(path + 'xvec.txt')  # interpolated vector
yvec = np.loadtxt(path + 'yvec.txt')  # interpolated vector
zvec = np.loadtxt(path + 'zvec.txt')  # interpolated vector

ze = np.loadtxt(path + 'ze.txt')  # interpolated vector

snew = np.loadtxt(path + 'snew.txt')  # new coordinates
yinterp = np.loadtxt(path + 'yinterp.txt')  # interpolation results
zinterp = np.loadtxt(path + 'zinterp.txt')  # interpolation results
yzinterp = np.loadtxt(path + 'yzinterp.txt')  # interpolation results

## Load Curvature
curv_original = np.loadtxt(path + 'curvature_original.txt')  # interpolation results
curv_bspline = np.loadtxt(path + 'curvature_bspline_interpolator.txt')

if __name__ == "__main__":
    plt.plot(xe, ye, 'r', alpha=0.5, lw=2, label=' noisy data')
    plt.plot(xvec, yvec, 'k-.', alpha=1, lw=2, label='original data')
    # plt.plot(xe, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(xe, yinterp, 'b', lw=3, alpha=0.4, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    plt.plot(xe, ze, 'r', alpha=0.5, lw=2, label=' noisy data')
    plt.plot(xvec, zvec, 'k-.', alpha=1, lw=2, label='original data')
    # plt.plot(xe, ye, 'rs', alpha=0.5, label='original data')
    plt.plot(xe, zinterp, 'b', lw=3, alpha=0.4, label='interpolated')
    # plt.plot(xe, yinterp, 'go', alpha=0.5, label='interpolated')
    plt.legend()
    plt.show()

    # MATRIX SMOOTHER

    plt.plot(yze[:, 0])
    plt.plot(yzinterp[:, 0])
    plt.title('Matrix Interpolation Results')
    plt.show()

    plt.plot(yze[:, 1])
    plt.plot(yzinterp[:, 1])

    plt.title('Matrix Interpolation Results')
    plt.show()

    # plot curvatures
    plt.plot(se, curv_original, 'r', lw=2, alpha=0.3, label='original curvature')
    plt.plot(se, curv_bspline, 'k-.', label='bspline curvature')
    plt.title('Curvature')
    plt.legend()
    plt.show()
