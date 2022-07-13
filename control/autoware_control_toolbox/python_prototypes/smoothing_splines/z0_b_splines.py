import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag

"""
    References:
    a00 Penalized Spline Regression and its Applications
"""
if __name__ == "__main__":
    ## Generate some data

    nx = 100
    x = np.linspace(0, 10, nx) + np.random.randn(nx) * 0
    y = np.sin(x) + np.random.randn(nx) * 0
    ynoisy = y + np.random.randn(nx) * 0.5

    ## Prepare arc-length parametrization
    dx = np.diff(x)
    dy = np.diff(y)

    s0 = np.hypot(dx, dy).cumsum()
    s0 = np.hstack((0, s0))

    ## Construct y-spline representation
    tvec = np.linspace(0, 1, nx)
    num_of_knots = int(nx * 0.3)
    knots = np.linspace(0, 1, num_of_knots)  # x is the interpolating coordinates

    bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3] + \
                                         [((t - kn) if (t - kn) >= 0 else 0.) ** 3 for kn in knots]

    # bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3]

    ## Construct B matrix
    B = np.array([bspline_primitive(ti, knots) for ti in tvec])  # t=0

    # Prepare data
    ydata = ynoisy

    ## Smoothing parameter
    smu = 1e-2

    ## Differentiating matrix
    mb, nb = B.shape

    ## FOR SECOND DERIVATIVEs second order D
    # D = np.eye(num_of_knots) - 2 * np.eye(num_of_knots, k=1) + np.eye(num_of_knots, k=2)
    # D = D[:-2, :]
    # D0 = np.zeros((4, 4))
    # Dc = block_diag(D0, D) # for second derivative

    ## For FIRST DERIVATIVE
    # D = np.eye(num_of_knots) - 1 * np.eye(num_of_knots, k=1)
    # D = D[:-1, :]
    # D0 = np.zeros((4, 4))
    # Dc = block_diag(D0, D)  # for second derivative

    ## For ZEROth ORDER Penalization
    D = np.eye(num_of_knots)
    D0 = np.zeros((4, 4))
    Dc = block_diag(D0, D)

    ## Now we can regress
    ## Without Weighting

    alpha_ridge = np.linalg.pinv(B.T @ B + (smu ** 2) * Dc.T @ Dc) @ B.T @ ydata
    # alpha_ridge = np.linalg.pinv(B.T @ B) @ B.T @ ydata

    ## now use this information
    nnew = 50
    tnew = np.linspace(0, 1, nnew)  # np.linspace(0, 1, 20)

    xnew = x[0] + (x[-1] - x[0]) * tnew

    yinterp = []

    for ti in tnew:
        btemp = bspline_primitive(ti, knots)

        yi = btemp @ alpha_ridge
        yinterp.append(yi)

    yinterp = np.array(yinterp)

    # plt.plot(x, y, 'r-.', label='original curve')
    plt.plot(x, y, 'r', alpha=1., lw=2, label='original curve')
    plt.plot(x, ynoisy, alpha=0.5, label='original noisy curve')
    plt.plot(xnew, yinterp, 'k-.', lw=1, label='B-splines Interpolated')
    plt.plot(xnew, yinterp, 'go', alpha=0.3, label='B-splines Interpolated')
    plt.legend()
    plt.show()

    a = 1
