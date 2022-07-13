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
    # N0 = 1.
    # N1 = t
    # N2 = t**2 and N3 = t**3  # t**2 and t**3

    kfinal0 = knots[-1]  # k[K]
    kfinal1 = knots[-2]  # k[K-1]

    dk = lambda t, ki: (max(0.0, (t - ki)) ** 3) / (kfinal0 - ki)
    dK_1 = lambda t, ki: (max(0.0, (t - kfinal1)) ** 3) / (kfinal0 - kfinal1)

    N4_1_K_2 = lambda t, ki: dk(t, ki) - dK_1(t, ki)
    basis = lambda t, ki: [1., t, t ** 2, t ** 3] + [N4_1_K_2(t, ki) for ki in knots[:-2]]  # + basis_final(t,
    # ki)  # k runs from 1:K-2

    return np.array(basis(t, knots))


def bspline_first_derivative(t, knots):
    '''
    From : Elements of statistical learning
    https://stats.stackexchange.com/questions/233232/the-definition-of-natural-cubic-splines-for-regression
    Only compute a single row [1, x, ...]
    :param t:
    :param knots:
    :return:
    '''

    kfinal0 = knots[-1]  # k[K]
    kfinal1 = knots[-2]  # k[K-1]

    dk = lambda t, ki: (3 * max(0.0, (t - ki)) ** 2) / (kfinal0 - ki)
    dK_1 = lambda t, ki: (3 * max(0.0, (t - kfinal1)) ** 2) / (kfinal0 - kfinal1)

    N4_1_K_2 = lambda t, ki: dk(t, ki) - dK_1(t, ki)

    basis = lambda t, ki: [0., 1., 2 * t, 3 * t ** 2] + [N4_1_K_2(t, ki) for ki in knots[:-2]]  # + basis_final(t,
    # ki)  # k runs from
    # 1:K-2

    return np.array(basis(t, knots))


def bspline_second_derivative(t, knots):
    '''
    From : Elements of statistical learning
    https://stats.stackexchange.com/questions/233232/the-definition-of-natural-cubic-splines-for-regression
    Only compute a single row [1, x, ...]
    :param t:
    :param knots:
    :return:
    '''

    kfinal0 = knots[-1]  # k[K]
    kfinal1 = knots[-2]  # k[K-1]

    dk = lambda t, ki: (6 * max(0.0, (t - ki))) / (kfinal0 - ki)
    dK_1 = lambda t, ki: (6 * max(0.0, (t - kfinal1))) / (kfinal0 - kfinal1)

    N4_1_K_2 = lambda t, ki: dk(t, ki) - dK_1(t, ki)

    basis = lambda t, ki: [0., 0., 2., 6 * t] + [N4_1_K_2(t, ki) for ki in
                                                 knots[:-2]]  # + basis_final(t, ki)  # k runs
    # from
    # 1:K-2

    return np.array(basis(t, knots))


def demmler_reinsch_ortho(B, D, alpha):
    '''
    https://statisticaloddsandends.wordpress.com/tag/smoothing-splines/
    :param B:
    :param D:
    :param alpha:
    :return:
    '''

    Dc = D.T @ D
    R = np.linalg.cholesky(B.T @ B + 0 * 1e-8 * Dc)
    Rinv = np.linalg.inv(R)

    U, S, Vt = np.linalg.svd(Rinv.T @ Dc @ Rinv)

    A = B @ Rinv @ U
    projection_mat = Rinv @ U @ np.linalg.inv((A.T @ A + alpha ** 2 * np.diag(S))) @ A.T

    return projection_mat  # projection_mat


def ls_qr(B, D, alpha):
    Ba = np.vstack((B, alpha * D))

    mb, nb = B.shape

    Q, R = np.linalg.qr(Ba)

    Q1 = Q[:mb, :]
    Rinv = np.linalg.inv(R)

    projection_mat = Rinv @ Q1.T

    return projection_mat


def curvature(rdot, rddot):
    # rdot = np.vstack((rdot, np.zeros((1, rdot.shape[1]))))
    # rddot = np.vstack((rddot, np.zeros((1, rddot.shape[1]))))

    curvature = []
    for k in range(rddot.shape[1]):
        cv = np.cross(rdot[:, k].flatten(), rddot[:, k].flatten()) / np.linalg.norm(rdot[:, k]) ** 3
        curvature.append(cv)

    return np.array(curvature)


def my_cls_solve(A, b, C, alpha):
    """

    :param A: Parameter matrix 1, t, t**2 etc
    :param b: Data vector
    :param C: Constraint mat
    :param d: is usually zero if we forumlate Cx = 0
    :return:
    """
    m, n = A.shape  # Ax = b, Cx = d Boyd pp 349 Intro to applied linear algebra
    p, n = C.shape
    Q, R = np.linalg.qr(np.vstack([A, alpha * C]))

    Q1 = Q[:m, :]
    Q2 = Q[m:m + p + 1, :]

    # Second QR factorization for Q2
    Qtil, Rtil = np.linalg.qr(Q2.T)

    ##
    '''

    Rtilde * w  = 2 Qtilde * Q1.transpose * b = 2 Rtilde_inv_transpose - 2*Rtilde_inv_transpose * d    
    Rtilde * w = b2
    '''
    b = b.reshape(-1, 1)
    b2 = 2 * Qtil.T @ (Q1.T @ b)
    w = np.linalg.pinv(Rtil) @ b2

    # Substitute we and solve another equation
    xhat = np.linalg.pinv(R) @ (Q1.T @ b - Q2.T @ w / 2)

    return xhat


if __name__ == "__main__":
    ## Generate some data
    noise_std = 0.1
    nx = 100

    k = 8
    c = 10

    tvec = np.linspace(0, 1, nx)  # t in [0, 1] in all interpolation scheme.

    x = k * tvec
    y = c * np.sin(x)

    xnoisy = x + (np.random.randn(nx) * noise_std * 1 + noise_std * 1) * 1 * 0
    ynoisy = y + (np.random.randn(nx) * noise_std * 1 + noise_std * 3) * 2 * 0

    ## Normalize data

    smean = np.mean(ynoisy)
    sstd = np.std(ynoisy)

    ## Prepare arc-length parametrization
    dx = np.diff(x)
    dy = np.diff(y)

    s0 = np.hypot(dx, dy).cumsum()
    s0 = np.hstack((0, s0))

    ## Construct y-spline representation

    num_of_knots = int(nx * 0.3)  # max(int(nx * 0.3), 20)
    knots = np.linspace(0, 1, num_of_knots)  # x is the interpolating coordinates

    bspline_primitive = lambda t, knots: [1., t, 0 * t ** 2, 0 * t ** 3] + \
                                         [((t - kn) if (t - kn) >= 0 else 0.) ** 3 for kn in knots]

    '''
        We need to reduce dimension of original data considering the computational time
        nc_size = curve fitting size. 
        nstride = 2, 3, .. jumping

    '''
    xdata = xnoisy
    ydata = ynoisy  # (ynoisy - smean) / sstd

    ## Construct B matrix
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
    # ProjectionMat = np.linalg.pinv((B.T @ B) + (smu ** 2) * Dc.T @ Dc) @ B.T
    # ProjectionMat = ls_qr(B, Dc, smu)

    ProjectionMat = demmler_reinsch_ortho(B, Dc, smu)

    ## Regress Bot x and y
    alpha_ridge_x = ProjectionMat @ xdata
    alpha_ridge_y = ProjectionMat @ ydata

    # alpha_ridge_x = my_cls_solve(B, xdata, Dc, smu)
    # alpha_ridge_y = my_cls_solve(B, ydata, Dc, smu)

    ## now use this information
    nnew = nx
    tnew = np.linspace(0, 1, nnew)  # np.linspace(0, 1, 20)

    xnew = x[0] + (x[-1] - x[0]) * tnew

    xinterp = []
    yinterp = []

    xdot = []
    ydot = []

    xddot = []
    yddot = []

    for ti in tnew:
        # btemp = bspline_primitive(ti, knots)

        btemp = bspline_basis(ti, knots)
        bdot = bspline_first_derivative(ti, knots)
        bddot = bspline_second_derivative(ti, knots)

        xi = btemp @ alpha_ridge_x
        yi = btemp @ alpha_ridge_y

        xid = bdot @ alpha_ridge_x
        yid = bdot @ alpha_ridge_y

        xidd = bddot @ alpha_ridge_x
        yidd = bddot @ alpha_ridge_y

        # yi = (yi * sstd + smean)  # unwrap
        xinterp.append(xi)
        yinterp.append(yi)

        xdot.append(xid)
        ydot.append(yid)

        xddot.append(xidd)
        yddot.append(yidd)

        ## Compute Curvature

    xinterp = np.array(xinterp)
    yinterp = np.array(yinterp)

    ydot = np.array(ydot)
    yddot = np.array(yddot)

    # plt.plot(x, y, 'r', alpha=1., lw=2, label='original curve')
    # plt.plot(x, ynoisy, alpha=0.8, label='original noisy curve')
    # plt.plot(xnew, yinterp, 'k-.', lw=1, label='B-splines Interpolated')
    # plt.plot(xnew, yinterp, 'go', alpha=0.3, label='B-splines Interpolated')
    # plt.legend()
    # plt.show()

    plt.plot(x, y, 'r', alpha=1., lw=2, label='original curve')
    plt.plot(xnoisy, ynoisy, alpha=0.8, label='original noisy curve')
    plt.plot(xinterp, yinterp, 'k-.', lw=1, label='B-splines Interpolated')
    # plt.plot(xinterp, yinterp, 'go', alpha=0.3, label='B-splines Interpolated')
    plt.legend()
    plt.show()

    ## dt COMPUTE ORIGINAL CURVATURE
    dt0 = tvec[2] - tvec[1]

    # dxdt = dx / dt0
    # dydt = dy / dt0
    # 
    # ddxdt = np.diff(dx) / dt0
    # ddydt = np.diff(dy) / dt0

    dxdt = np.zeros_like(x) + k
    ddxdt = np.zeros_like(x)

    dydt = c * k * np.cos(x)
    ddydt = -c * k ** 2 * np.sin(x)

    r0dot = np.vstack((dxdt, dydt))
    r0ddot = np.vstack((ddxdt, ddydt))

    ## Curvature
    curv0 = curvature(r0dot, r0ddot)

    ## COMPUTE SPLINE CURVATURE
    dts = tnew[2] - tnew[1]
    rsdot = np.vstack((xdot, ydot))
    rsddot = np.vstack((xddot, yddot))

    curvsp = curvature(rsdot, rsddot)

    plt.plot(tvec, curv0, 'r-.', alpha=0.6, lw=2, label='original curvature')
    plt.plot(tnew, curvsp, label='spline curvature')
    plt.legend()
    plt.show()

    # dydt = dydt - dydt[0]
    # plt.plot(tvec[:-1], dydt, label='original ydot')
    # plt.plot(tnew, ydot, label='spline ydot')
    # plt.legend()
    # plt.show()

    a = 1
