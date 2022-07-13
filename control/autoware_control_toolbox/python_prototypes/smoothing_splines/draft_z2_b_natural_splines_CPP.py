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

    basis_final = lambda t, ki: [N4_1_K(t, knots[-1])]
    basis = lambda t, ki: [1., t] + [N4_1_K_2(t, ki) for ki in knots[:-1]] + basis_final(t, ki)  # k runs from 1:K-2

    # thetal*(x-ki) at the final point thetaK = 1/(xi[K] -xi[K-1]) - 1/(xi[K-1] - xil)

    return basis(t, knots)


def bspline_second_derivative(t, knots):
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

    dk = lambda t, ki: (6 * max(0.0, (t - ki)) - 6 * max(0.0, (t - kfinal0))) / (kfinal0 - ki)
    dK_1 = lambda t, ki: (max(0.0, (t - kfinal1)) * 6 - max(0.0, (t - kfinal0)) * 6) / (kfinal0 - kfinal1)
    dK_2 = lambda t, ki: (max(0.0, (t - kfinal2)) * 6 - max(0.0, (t - kfinal0)) * 6) / (kfinal0 - kfinal2)

    N4_1_K_2 = lambda t, ki: dk(t, ki) - dK_1(t, ki)
    N4_1_K = lambda t, ki: dK_1(t, ki) - dK_2(t, ki)

    basis_final = lambda t, ki: [N4_1_K(t, knots[-1])]
    basis = lambda t, ki: [0., 0.] + [N4_1_K_2(t, ki) for ki in knots[:-1]] + basis_final(t, ki)  # k runs from 1:K-2

    # thetal*(x-ki) at the final point thetaK = 1/(xi[K] -xi[K-1]) - 1/(xi[K-1] - xil)

    return basis(t, knots)


def bspline_first_derivative(t, knots):
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

    dk = lambda t, ki: (3 * max(0.0, (t - ki)) ** 2 - 3 * max(0.0, (t - kfinal0)) ** 2) / (kfinal0 - ki)
    dK_1 = lambda t, ki: (3 * max(0.0, (t - kfinal1)) ** 2 - 3 * max(0.0, (t - kfinal0)) ** 2) / (kfinal0 - kfinal1)
    dK_2 = lambda t, ki: (3 * max(0.0, (t - kfinal2)) ** 2 - 3 * max(0.0, (t - kfinal0)) ** 2) / (kfinal0 - kfinal2)

    N4_1_K_2 = lambda t, ki: dk(t, ki) - dK_1(t, ki)
    N4_1_K = lambda t, ki: dK_1(t, ki) - dK_2(t, ki)

    basis_final = lambda t, ki: [N4_1_K(t, knots[-1])]
    basis = lambda t, ki: [0., 0.] + [N4_1_K_2(t, ki) for ki in knots[:-1]] + basis_final(t, ki)  # k runs from 1:K-2

    # thetal*(x-ki) at the final point thetaK = 1/(xi[K] -xi[K-1]) - 1/(xi[K-1] - xil)

    return basis(t, knots)


def demmler_reinsch_ortho(B, D):
    R = np.linalg.cholesky(B.T @ B + 1e-8 * D.T @ D)

    projection_mat = 1

    return projection_mat


def curvature(r, rdot, rddot):
    pass


if __name__ == "__main__":
    ## Generate some data
    noise_std = 0.1
    nx = 10
    x = np.linspace(0, 8, nx) + np.random.randn(nx) * 0
    y = 10 * np.sin(x) + np.random.randn(nx) * 0

    xnoisy = x + np.random.randn(nx) * noise_std * 2 + noise_std * 1
    ynoisy = y + np.random.randn(nx) * noise_std * 2 + noise_std * 1

    ## Normalize data

    smean = np.mean(ynoisy)
    sstd = np.std(ynoisy)

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
    xdata = xnoisy
    ydata = ynoisy  # (ynoisy - smean) / sstd

    ## Construct B matrix
    # B = np.array([bspline_primitive(ti, knots) for ti in tvec_strided])  # t=0
    B = np.array([bspline_basis(ti, knots) for ti in tvec])

    ## Smoothing parameter
    smu = noise_std ** 3

    ## Differentiating matrix
    mb, nb = B.shape

    # FOR SECOND DERIVATIVEs second order D
    Dc = np.array([bspline_second_derivative(ti, knots) for ti in tvec])

    ## Now we can regress
    ## Without Weighting

    W = np.eye(nb) / 1 ** 2

    # Instead of PINV we can use Demmler-Reisch
    ProjectionMat = np.linalg.pinv((B.T @ B) + (smu ** 2) * Dc.T @ Dc) @ B.T
    ProjectionMat = demmler_reinsch_ortho(B, Dc)

    ## Regress Bot x and y
    alpha_ridge_x = ProjectionMat @ xdata
    alpha_ridge_y = ProjectionMat @ ydata

    ## now use this information
    nnew = nx
    tnew = np.linspace(0, 1, nnew)  # np.linspace(0, 1, 20)

    xnew = x[0] + (x[-1] - x[0]) * tnew

    xinterp = []
    yinterp = []

    for ti in tnew:
        # btemp = bspline_primitive(ti, knots)

        btemp = bspline_basis(ti, knots)

        xi = btemp @ alpha_ridge_x
        yi = btemp @ alpha_ridge_y

        # yi = (yi * sstd + smean)  # unwrap
        xinterp.append(xi)
        yinterp.append(yi)

        ## Compute Curvature

    xinterp = np.array(xinterp)
    yinterp = np.array(yinterp)

    # plt.plot(x, y, 'r', alpha=1., lw=2, label='original curve')
    # plt.plot(x, ynoisy, alpha=0.8, label='original noisy curve')
    # plt.plot(xnew, yinterp, 'k-.', lw=1, label='B-splines Interpolated')
    # plt.plot(xnew, yinterp, 'go', alpha=0.3, label='B-splines Interpolated')
    # plt.legend()
    # plt.show()

    plt.plot(x, y, 'r', alpha=1., lw=2, label='original curve')
    plt.plot(xnoisy, ynoisy, alpha=0.8, label='original noisy curve')
    plt.plot(xinterp, yinterp, 'k-.', lw=1, label='B-splines Interpolated')
    plt.plot(xinterp, yinterp, 'go', alpha=0.3, label='B-splines Interpolated')
    plt.legend()
    plt.show()

    a = 1
