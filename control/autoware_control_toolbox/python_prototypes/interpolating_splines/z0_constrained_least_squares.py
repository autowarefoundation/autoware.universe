import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag

np.printoptions(precision=3, suppress='true')


def get_t(nx, t=0):
    tprimitive_vec = [1, t, t ** 2, t ** 3]
    T0 = block_diag(*[tprimitive_vec] * (nx - 1))

    return T0


def fT(nx):
    '''
        nx number of data points in x, y
        :return: given  t parameter construct least squares T
    '''

    mx_1 = nx - 1

    t = 0
    tprimitive_vec = [1, t, t ** 2, t ** 3]
    T0 = block_diag(*[tprimitive_vec] * mx_1)

    t = 1
    tprimitive_vec = [1, t, t ** 2, t ** 3]
    T1 = block_diag(*[tprimitive_vec] * mx_1)

    T = np.vstack((T0, T1))

    f_primitive = np.array([[1, 1, 1, 1, -1, 0, 0, 0], [0, 1, 2, 3, 0, -1, 0, 0]])
    blockf0 = f_primitive[:, 0:4]  # np.array([1, 1, 1, 1, -1, 0, 0, 0]).reshape(1, -1)
    blockf1 = f_primitive[:, 4:]  # np.array([0, 1, 2, 3, 0, -1, 0, 0]).reshape(1, -1)

    ## number of eta η

    eyef0 = np.eye(mx_1, k=0)
    eyef0 = eyef0[:-1, :]

    eyef1 = np.eye(mx_1, k=0)
    eyef1 = eyef1[1:, :]

    F0 = np.kron(eyef0, blockf0)
    F1 = np.kron(eyef1, blockf1)

    F = F0 + F1

    # Repeat tprimitive
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


if __name__ == "__main__":

    ## DEMO

    # compare with KKT method
    m = 10
    n = 5
    p = 2
    A = np.random.normal(size=(m, n))
    b = np.random.normal(size=m)
    C = np.random.normal(size=(p, n))
    d = np.random.normal(size=p)

    # xKKT = cls_solve_kkt(A, b, C, d)
    xQR = cls_solve(A, b, C, d)
    xQRmy = my_cls_solve(A, b, C, d)

    # compare solutions
    print("my solution works if this diff is small ", np.linalg.norm(xQRmy - xQR))

    nx = 50
    x = np.linspace(0, 10, nx) + np.random.randn(nx) * 0.1
    y = np.sin(x) + np.random.randn(nx) * 0.1
    # y = 10 + 0.04 * x + 0.007 * x ** 2 - 0.002 * x ** 3 + np.random.randn(nx)

    x = x / np.max(np.abs(x))
    y = y / np.max(np.abs(y))

    plt.plot(x, y)
    plt.show()

    ## Get shape of x and y
    '''
        Reference Mumin -gmail
    '''

    A, C = fT(nx=nx)  # due to the data matrix construction we need nx-1 size of T and F

    mm, nn = C.shape
    dc = np.zeros((mm, 1))

    ## Data Matrix X [x(0), x(1)] just repeat
    xdata = np.hstack((x[:-1], x[1:])).reshape(-1, 1)
    ydata = np.hstack((y[:-1], y[1:])).reshape(-1, 1)

    # Solve by QR

    eta_x = my_cls_solve(A, xdata, C, dc)
    eta_y = my_cls_solve(A, ydata, C, dc)
    a = 1

    ## Plot if works

    Tint = get_t(nx=nx, t=0.1)
    xinterp = Tint @ eta_x
    yinterp = Tint @ eta_y

    plt.plot(x, '-.r', label="x original")
    plt.plot(xinterp.flatten(), 'g', alpha=0.6, label="x_interpolated")
    plt.legend()
    plt.show()

    plt.plot(y, '-.r', label="y original")
    plt.plot(yinterp.flatten(), 'g', alpha=0.6, label="y_interpolated")
    plt.legend()
    plt.show()
