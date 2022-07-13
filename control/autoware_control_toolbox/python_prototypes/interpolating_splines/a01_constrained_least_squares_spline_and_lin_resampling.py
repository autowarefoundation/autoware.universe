import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag

np.printoptions(precision=3, suppress='true')
from binary_index_search import binary_search


def get_t(nx, t=0, interpolation_type='spline'):
    if interpolation_type == 'spline':
        tprimitive_vec = [1, t, t ** 2, t ** 3]
        T0 = block_diag(*[tprimitive_vec] * (nx - 1))

    elif interpolation_type == 'line':
        tprimitive_vec = [1, t]
        T0 = block_diag(*[tprimitive_vec] * (nx - 1))

    return T0


def create_vandermonde(nx):
    '''

    [1, t0, t0**2, ..] --> t
    [1, t1, t1**2, ..]
    :param t: vector
    :param nx: number of data points
    :return:
    '''

    t = np.linspace(0, nx, nx + 1)
    a = 1


def fT(nx, interpolation_type='spline'):
    '''
        nx number of data points in x, y
        :return: given  t parameter construct least squares T
    '''

    mx_1 = nx - 1  # number of intervals

    if interpolation_type == 'spline':
        t = 0
        tprimitive_vec = [1, t, t ** 2, t ** 3]
        T0 = block_diag(*[tprimitive_vec] * mx_1)

        t = 1
        tprimitive_vec = [1, t, t ** 2, t ** 3]
        T1 = block_diag(*[tprimitive_vec] * mx_1)

        T = np.vstack((T0, T1))

        f_primitive = np.array([[1, 1, 1, 1, -1, 0, 0, 0], [0, 1, 2, 3, 0, -1, 0, 0], [0, 0, 2, 6, 0, 0, -2, 0]])
        blockf0 = f_primitive[:, 0:4]  # np.array([1, 1, 1, 1, -1, 0, 0, 0]).reshape(1, -1)
        blockf1 = f_primitive[:, 4:]  # np.array([0, 1, 2, 3, 0, -1, 0, 0]).reshape(1, -1)


    elif interpolation_type == 'line':
        t = 0
        tprimitive_vec = [1, t]
        T0 = block_diag(*[tprimitive_vec] * mx_1)

        t = 1
        tprimitive_vec = [1, t]
        T1 = block_diag(*[tprimitive_vec] * mx_1)

        T = np.vstack((T0, T1))

        f_primitive = np.array([[1, 1, -1, 0]])
        blockf0 = f_primitive[:, 0:2]  # np.array([1, 1, 1, 1, -1, 0, 0, 0]).reshape(1, -1)
        blockf1 = f_primitive[:, 2:]  # np.array([0, 1, 2, 3, 0, -1, 0, 0]).reshape(1, -1)

        a = 1

        ## Loop stops
    eyef0 = np.eye(mx_1, k=0)
    eyef0 = eyef0[:-1, :]

    eyef1 = np.eye(mx_1, k=0)
    eyef1 = eyef1[1:, :]

    F0 = np.kron(eyef0, blockf0)
    F1 = np.kron(eyef1, blockf1)
    F = F0 + F1

    ## sc is for scaling the time, 

    return T, F


def back_subst(R, b_tilde):
    n = R.shape[0]
    x = np.zeros(n)

    for i in reversed(range(n)):
        x[i] = b_tilde[i]

        for j in range(i + 1, n):
            x[i] = x[i] - R[i, j] * x[j]

        x[i] = x[i] / R[i, i]

    return x


def solve_via_backsub(A, b):
    Q, R = np.linalg.qr(A)
    b_tilde = Q.T @ b

    x = back_subst(R, b_tilde)

    return x


# QR factorization CONSTRAINTED LEAST SQUARES
def cls_solve(A, b, C, d):
    m, n = A.shape  # Ax = b, Cx = d Boyd pp 349 Intro to applied linear algebra
    p, n = C.shape
    Q, R = np.linalg.qr(np.vstack([A, C]))

    Q1 = Q[:m, :]
    Q2 = Q[m:m + p + 1, :]

    # Second QR factorization for Q2
    Qtil, Rtil = np.linalg.qr(Q2.T)

    ##
    '''
    
    Rtilde * w  = 2 Qtilde * Q1.transpose * b = 2 Rtilde_inv_transpose - 2*Rtilde_inv_transpose * d    
    Rtilde * w = b2
    '''

    w = solve_via_backsub(Rtil, (2 * Qtil.T @ (Q1.T @ b) - 2 * solve_via_backsub(Rtil.T, d)))
    xhat = solve_via_backsub(R, (Q1.T @ b - Q2.T @ w / 2))

    return xhat


def my_cls_solve(A, b, C, d):
    """


    :param A: Parameter matrix 1, t, t**2 etc
    :param b: Data vector
    :param C: Constraint mat
    :param d: is usually zero if we forumlate Cx = 0
    :return:
    """
    m, n = A.shape  # Ax = b, Cx = d Boyd pp 349 Intro to applied linear algebra
    p, n = C.shape
    Q, R = np.linalg.qr(np.vstack([A, C]))

    Q1 = Q[:m, :]
    Q2 = Q[m:m + p + 1, :]

    # Second QR factorization for Q2
    Qtil, Rtil = np.linalg.qr(Q2.T)

    ##
    '''

    Rtilde * w  = 2 Qtilde * Q1.transpose * b = 2 Rtilde_inv_transpose - 2*Rtilde_inv_transpose * d    
    Rtilde * w = b2
    '''

    b2 = 2 * Qtil.T @ (Q1.T @ b) - 2 * np.linalg.inv(Rtil.T) @ d
    w = np.linalg.pinv(Rtil) @ b2

    # Substitute we and solve another equation
    xhat = np.linalg.pinv(R) @ (Q1.T @ b - Q2.T @ w / 2)

    return xhat


def piecewise_spline_func(t, coeffs, interp_type='spline'):
    if interp_type == 'spline':
        tt = np.array([1, t, t ** 2, t ** 3])

    elif interp_type == 'line':
        tt = np.array([1, t])

    feval = np.dot(tt, coeffs.flatten())

    return feval


def interp_spline(tvec, eta, interp_type='spline'):
    '''

    splines has 4 coefficients a, b, c, d
    :param tvec: new interpolation points
    :param eta: interpolation coefficients
    :return:
    '''

    ns = 2 if interp_type == 'line' else 4  # number of splien coeffs

    # find tvec's interval
    nstart = tvec[0]
    nend = tvec[-1]

    xinterp = []
    for ti in tvec:
        print(ti)

        if int(ti) == nend:
            zleft = int(nend - 1)
            zright = int(nend)

        else:
            zleft, zright = binary_search(ti, nstart, nend)

        indstart = zleft * ns
        indend = zright * ns

        # pick the coeffs at this interval
        piece_coeffs = eta[indstart:indend]

        ## compute the dt of this ti
        dt = ti - zleft

        assert (dt <= 1 and dt >= 0)

        xval = piecewise_spline_func(dt, piece_coeffs, interp_type)

        ## append to xinterp
        xinterp.append(xval)

    return np.array(xinterp)


if __name__ == "__main__":
    ## DEMO
    interp_type = 'line'
    nx = 100
    x = np.linspace(0, 10, nx) + np.random.randn(nx) * 0.0
    y = np.sin(x) + np.random.randn(nx) * 0.0
    # y = 10 + 0.04 * x + 0.007 * x ** 2 - 0.002 * x ** 3 + np.random.randn(nx)

    # x = x / np.max(np.abs(x))
    # y = y / np.max(np.abs(y))

    # plt.plot(x, y)
    # plt.show()

    '''
        A theta = b -- > b is data matrix, theta is the coefficients, A is A(t) functions of paremetrization
        C is the constraint matrix. 

        Ax = b
        Cx = d 
    '''

    A, C = fT(nx=nx, interpolation_type=interp_type)  # due to the data matrix construction we need nx-1 size of T and F

    mm, nn = C.shape
    dc = np.zeros((mm, 1))  # constraint matrix Cx = d where d is usually zero.

    ## Data Matrix X [x(0), x(1)] just repeat
    xdata = np.hstack((x[:-1], x[1:])).reshape(-1, 1)
    ydata = np.hstack((y[:-1], y[1:])).reshape(-1, 1)

    # Solve by QR

    eta_x = my_cls_solve(A, xdata, C, dc)
    eta_y = my_cls_solve(A, ydata, C, dc)
    a = 1

    ## Plot if works
    Tint = get_t(nx=nx, t=0.1, interpolation_type=interp_type)
    xinterp = Tint @ eta_x
    yinterp = Tint @ eta_y

    plt.plot(x, y, 'r-.', label='original curve')
    plt.plot(xinterp, yinterp, 'g', alpha=0.6, label='interpolated curve')
    plt.show()

    # plt.plot(x, '-.r', label="x original")
    # plt.plot(xinterp.flatten(), 'g', alpha=0.6, label="x_interpolated")
    # plt.legend()
    # plt.show()
    #
    # plt.plot(y, '-.r', label="y original")
    # plt.plot(yinterp.flatten(), 'g', alpha=0.6, label="y_interpolated")
    # plt.legend()
    # plt.show()

    ## Interpolate new data points
    '''
        we have nx data point and nx-1 intervals
    '''
    tvec_t0_be_interpolated = np.linspace(0, nx - 1, 30)

    xinterp = interp_spline(tvec_t0_be_interpolated, eta_x, interp_type)
    yinterp = interp_spline(tvec_t0_be_interpolated, eta_y, interp_type)

    plt.plot(x, y, 'r-.', label='original curve')
    plt.plot(xinterp, yinterp, 'go', alpha=0.6, label='interpolated curve')
    plt.show()

    ##
    # plt.plot(y, 'r-.', label='original x ')
    # plt.plot(yinterp, 'go', alpha=0.6, label='interpolated x')
    # plt.show()
    aa = 1
