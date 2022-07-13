import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag

"""
    References:
    a00 Penalized Spline Regression and its Applications
"""

'''
 We construct new basis functions
'''


def bspline_basis(t, knots):
    '''
    From : Elements of statistical learning
    https://stats.stackexchange.com/questions/233232/the-definition-of-natural-cubic-splines-for-regression
    Only compute a single row [1, x, ...]
    :param t:
    :param knots:
    :return:
    '''

    # [1., t, 0 * t ** 2, 0 * t ** 3 + ((t - kn) if (t - kn) >= 0 else 0.) ** 3 for kn in knots]
    N0 = 1.
    N1 = t
    N2 = N3 = 0  # t**2 and t**3

    kfinal0 = knots[-1]  # k[K]
    kfinal1 = knots[-2]  # k[K-1]
    kfinal2 = knots[-3]  # k[K-1]

    dk = lambda t, ki: (max(0.0, (t - ki)) ** 3 - max(0.0, (t - kfinal0)) ** 3) / (kfinal0 - ki)
    dK_1 = lambda t, ki: (max(0.0, (t - kfinal1)) ** 3 - max(0.0, (t - kfinal0)) ** 3) / (kfinal0 - kfinal1)
    dK_2 = lambda t, ki: (max(0.0, (t - kfinal2)) ** 3 - max(0.0, (t - kfinal0)) ** 3) / (kfinal0 - kfinal2)

    N4_1_K_2 = lambda t, ki: dk(t, ki) - dK_1(t, ki)
    N4_1_K = lambda t, ki: dK_1(t, ki) - dK_2(t, ki)

    # dk-2 - dk-1
    # final basis for the final node is zero
    dK_1 = lambda t, ki: (max(0.0, (t - kfinal1)) ** 3 - max(0.0, (t - kfinal0)) ** 3) / (kfinal0 - kfinal1)

    basis_final = lambda t, ki: [N4_1_K(t, knots[-1])]
    basis = lambda t, ki: [1., t] + [N4_1_K_2(t, ki) for ki in knots[:-1]] + basis_final(t, ki)  # k runs from 1:K-2

    # thetal*(x-ki) at the final point thetaK = 1/(xi[K] -xi[K-1]) - 1/(xi[K-1] - xil)

    return basis(t, knots)


if __name__ == "__main__":
    ## Generate some data
    noise_std = 0.1
    nx = 80
    x = np.linspace(0, 8, nx) + np.random.randn(nx) * 0
    y = 10 * np.sin(x) + np.random.randn(nx) * 0
    ynoisy = y + np.random.randn(nx) * noise_std * 10 + noise_std * 5

    ## Prepare arc-length parametrization
    dx = np.diff(x)
    dy = np.diff(y)

    s0 = np.hypot(dx, dy).cumsum()
    s0 = np.hstack((0, s0))

    ## Construct y-spline representation
    tvec = np.linspace(0, 1, nx)  # t in [0, 1] in all interpolation scheme.
    num_of_knots = int(nx * 0.3)
    knots = np.linspace(0, 1, num_of_knots)  # x is the interpolating coordinates

    bspline_primitive = lambda t, knots: [1., t, 0 * t ** 2, 0 * t ** 3] + \
                                         [((t - kn) if (t - kn) >= 0 else 0.) ** 3 for kn in knots]

    # bspline_primitive = lambda t, knots: [1., t, t ** 2, t ** 3]

    '''
        We need to reduce dimension of original data considering the computational time
        nc_size = curve fitting size. 
        nstride = 2, 3, .. jumping
    
    '''
    nstride = 1
    tvec_strided = tvec[::nstride]
    ydata_strided = ynoisy[::nstride]

    ## Construct B matrix
    # B = np.array([bspline_primitive(ti, knots) for ti in tvec_strided])  # t=0
    B = np.array([bspline_basis(ti, knots) for ti in tvec_strided])

    ## Smoothing parameter
    smu = noise_std ** 3

    ## Differentiating matrix
    mb, nb = B.shape

    # FOR SECOND DERIVATIVEs second order D
    D = np.eye(num_of_knots) - 2 * np.eye(num_of_knots, k=1) + np.eye(num_of_knots, k=2)
    D = D[:-2, :]
    D0 = np.zeros((2, 2))
    Dc = block_diag(D0, D)  # for second derivative

    ## For FIRST DERIVATIVE
    # D = np.eye(num_of_knots) - 1 * np.eye(num_of_knots, k=1)
    # D = D[:-1, :]
    # D0 = np.zeros((4, 4))
    # Dc = block_diag(D0, D)  # for second derivative

    # ## For ZEROth ORDER Penalization
    # D = np.eye(num_of_knots)
    # D0 = np.zeros((4, 4))
    # Dc = block_diag(D0, D)

    ## Now we can regress
    ## Without Weighting

    W = np.eye(nb) / 1 ** 2
    ProjectionMat = np.linalg.pinv((B.T @ B) + (smu ** 2) * Dc.T @ Dc) @ B.T
    alpha_ridge = ProjectionMat @ ydata_strided

    ## now use this information
    nnew = nx
    tnew = np.linspace(0, 1, nnew)  # np.linspace(0, 1, 20)

    xnew = x[0] + (x[-1] - x[0]) * tnew

    yinterp = []

    for ti in tnew:
        # btemp = bspline_primitive(ti, knots)
        btemp = bspline_basis(ti, knots)

        yi = btemp @ alpha_ridge
        yinterp.append(yi)

    yinterp = np.array(yinterp)
    # plt.plot(x, y, 'r-.', label='original curve')
    plt.plot(x, y, 'r', alpha=1., lw=2, label='original curve')
    plt.plot(x, ynoisy, alpha=0.8, label='original noisy curve')
    plt.plot(xnew, yinterp, 'k-.', lw=1, label='B-splines Interpolated')
    plt.plot(xnew, yinterp, 'go', alpha=0.3, label='B-splines Interpolated')
    plt.legend()
    plt.show()

    a = 1
