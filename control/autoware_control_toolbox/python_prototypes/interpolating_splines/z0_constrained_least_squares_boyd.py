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


if __name__ == "__main__":

    nx = 50
    x = np.linspace(0, 10, nx) + np.random.randn(nx) * 0.05 * 0
    y = np.sin(x) + np.random.randn(nx) * 0.05 * 0
    # y = 10 + 0.04 * x + 0.007 * x ** 2 - 0.002 * x ** 3 + np.random.randn(nx)

    x = x / np.max(np.abs(x))
    y = y / np.max(np.abs(y))

    plt.plot(x, y)
    plt.show()

    ## Get shape of x and y
    '''
        Reference Mumin -gmail    
    '''

    P, F = fT(nx=nx)  # due to the data matrix construction we need nx-1 size of T and F

    ## Data Matrix X [x(0), x(1)] just repeat
    xdata = np.hstack((x[:-1], x[1:])).reshape(-1, 1)
    ydata = np.hstack((y[:-1], y[1:])).reshape(-1, 1)

    ## Ordinary Least Square
    Pt = P.T @ P
    pinvPt = np.linalg.pinv(Pt)
    Tprojection = pinvPt @ P.T

    eta_x_ols = Tprojection @ xdata
    eta_y_ols = Tprojection @ ydata

    ## Interpolate
    Tint = get_t(nx, t=0.0)
    xinterp = Tint @ eta_x_ols
    yinterp = Tint @ eta_y_ols

    plt.plot(x, '-.r', label="x original")
    plt.plot(xinterp.flatten(), 'g', alpha=0.6, label="x_interpolated")
    plt.legend()
    plt.show()

    plt.plot(y, '-.r', label="y original")
    plt.plot(yinterp.flatten(), 'g', alpha=0.6, label="y_interpolated")
    plt.legend()
    plt.show()

    ## 

    ## COMPUTE CONSTRAINED.
    pinvTTF = pinvPt @ F.T
    # eta_x_cs = eta_x_unconstrained - pinvTTF @ np.linalg.pinv(F @ pinvTTF) @ F @ eta_x_unconstrained
    # eta_y_cs = eta_y_unconstrained - pinvTTF @ np.linalg.pinv(F @ pinvTTF) @ F @ eta_y_unconstrained
    hh_1ft = np.linalg.pinv(P.T @ P) @ F.T

    eta_x_cs = eta_x_ols - hh_1ft @ np.linalg.pinv(F @ hh_1ft) @ F @ eta_x_ols
    eta_y_cs = eta_y_ols - hh_1ft @ np.linalg.pinv(F @ hh_1ft) @ F @ eta_y_ols

    a = 1

    Tint = get_t(nx=nx, t=1)
    xinterp = Tint @ eta_x_cs
    yinterp = Tint @ eta_y_cs

    plt.plot(x, '-.r', label="x original")
    plt.plot(xinterp.flatten(), 'g', alpha=0.6, label="x_interpolated")
    plt.legend()
    plt.show()

    plt.plot(y, '-.r', label="y original")
    plt.plot(yinterp.flatten(), 'g', alpha=0.6, label="y_interpolated")
    plt.legend()
    plt.show()
