import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag

np.printoptions(precision=3, suppress='true')


class PCG():
    def __init__(self):
        '''

        :param A: Original system A to be decomposed
        :param b: Original system b
        '''

        self.A = None
        self.b = None

        self.L = None
        self.M = None
        self.Minv = None

        self.Ahat = None
        self.bhat = None

        self.maxiter = 100
        self.convergence_threshold = 1e-8

    def set_M_Ab(self, A, b):
        self.A = A
        self.b = b

        self.M = np.diag(np.diag(A))
        self.Minv = np.diag(1 / np.diag(A))
        self.L = np.sqrt(np.diag(self.Minv)).reshape(-1, 1)  # Columnself.L @ self.L.T  # inv(M)A = inv(M)b

        self.Ahat = self.L.T @ A @ self.L  # Ahat = L'@A@L
        self.bhat = self.L.T @ b  # bhat = L'@b

    def isConvergeredL1(self, rn):
        rmax = np.sum(np.abs(rn))

        print(rmax)
        return rmax < self.convergence_threshold

    def diag_matrix_product(self, diag_matrix, vector):
        # for fast C++ implementation
        diag_vec = np.diag(diag_matrix).reshape(-1, 1)  # column vector
        results = np.zeros_like(diag_vec)

        for k in range(diag_matrix.shape[0]):
            results[k] = diag_vec[k] * vector[k]

        return results  # column vector

    def solve(self):
        '''
        Chapter 16 Preconditioning
        :return:
        '''

        # prepare x0 = 0 
        xn = np.zeros((self.A.shape[0], 1))  # column vector x0 initial estimate
        rn = self.b  # r0 initial residual r = b- Ax

        # Compute rtilda_0
        rtilda_n = self.Minv @ rn  # rtilda_0 -> transformed problem residual
        pn = rtilda_n  # initial direction

        # compute the step length
        alpha_n = rn.T @ rtilda_n / (pn.T @ self.A @ pn)  # scalar

        # update the approximate solution
        xn = xn + alpha_n * pn

        # update original equation residuals rn = b - Ax
        rn_prev = rn
        rn = rn - alpha_n * self.A @ pn

        # plt.spy(self.Minv)
        # plt.show()

        for k in range(self.maxiter):
            # solve for rtilda = Minv @ rn
            rtilda_n_prev = rtilda_n
            rtilda_n = self.diag_matrix_product(self.Minv, rn)  # self.Minv @ rn  # update rtilda

            # compute gradient direction correction factor
            beta_n = rn.T @ rtilda_n / (rn_prev.T @ rtilda_n_prev)

            # Update search direction
            pn = rtilda_n + beta_n * pn

            # Compute alpha
            alpha_n = rn.T @ rtilda_n / (pn.T @ self.A @ pn)  # scalar

            # update the approximate solution
            xn = xn + alpha_n * pn

            ## UPDATE RESIDUALS
            # update original equation residuals rn = b - Ax
            rn_prev = rn
            rn = rn - alpha_n * self.A @ pn

            if self.isConvergeredL1(rn):
                break

        print(f"Converged after {k} steps")
        return xn


class autowareSpline():
    def __init__(self, interpolation_type='spline'):
        self.interp_type = interpolation_type;

        self.A = None
        self.b = None
        self.tvec = None

        self.PCG = PCG()  # PCG solver zero initialization
        self.coeffs = None

    def interpolate(self, tnew, tbase, ybase):
        self.tvec = tbase
        self.set_coefficients(ybase)

        ## Number of coefficients
        ns = {'line': 2, 'spline': 4}.get(self.interp_type)
        tvec = tnew

        # find out in which interval tvec[i] lives
        assert tvec[0] >= self.tvec[0] and tvec[-1] <= self.tvec[-1], "Interpolation is requested, please check the " \
                                                                      "interpolating range"

        xinterp = []  # Prepare the interpolated value

        ## CALL BINARY SEARCH
        for ti in tvec:

            ind_left, ind_right = self.binary_search(ti)

            '''
                ind_left and ind_right is the time or arc length interval start and end .
                since we use [0, 1] interval for all parametrization, we need to normalize each itnerval. 
            '''
            ## Polynomial indices start and end
            indstart = ind_left
            indend = ind_right

            # Pick the coeffs at this interval

            piece_coeffs = self.coeffs[indstart, :]

            ## Compute the dt of this ti;
            dt = ti - self.tvec[ind_left]  # ti is always greater than ind_left.
            dt = dt / (self.tvec[ind_right] - self.tvec[ind_left])  # normalize

            assert dt <= 1 and dt >= 0, "dt must be in [0, 1]"

            xval = self.piecewise_func(dt, piece_coeffs)

            ## append to xinterp
            xinterp.append(xval)

        return np.array(xinterp)

    def set_coefficients(self, y0):
        ## We solve for c in [a, b, c, d]
        # Set C coefficents.

        '''
                Cmat * c = f(a) right handside (A * x = b)
                Without or With USING matrices
        '''
        # there are nx equations including c0 and cn two times with n-1 intervals. n-2 second derivatives.

        cprimitive = np.array([1., 4., 1]).reshape(1, 3)
        Cmat = np.zeros((nx, nx))
        Cmat[0, 0] = 1
        Cmat[nx - 1, nx - 1] = 1

        for k in range(1, nx - 1):
            Cmat[k, k - 1:k - 1 + cprimitive.shape[1]] = cprimitive

        # print(Cmat)

        # COMPUTE RHS as a function of data
        brhs = np.zeros((nx, 1))

        for k in range(1, nx - 1):
            brhs[k] = 3 * (y0[k - 1] - 2 * y0[k] + y0[k + 1])

        self.A = Cmat
        self.b = brhs  # Equation RHS

        ## set PCG object M, A, b.
        self.PCG.set_M_Ab(self.A, self.b)

        if self.interp_type == 'spline':
            a = y0.reshape(-1, 1)  # coefficients a of [a, b, c, d]
            c = self.PCG.solve()

            d = np.zeros_like(c)
            d[:-1] = (c[1:] - c[:-1]) / 3  # last element of d is set to zero

            b = np.zeros_like(c)
            b[:-1] = (a[1:] - a[0:-1]) - (c[1:] + 2 * c[:-1]) / 3
            b[-1] = b[-2] + 2 * c[-2] + 3 * d[-2]  # set the last

            self.coeffs = np.hstack((a, b, c, d))

        elif self.interp_type == 'line':
            pass

    def binary_search(self, search_val_i) -> (int, int):
        # locate the interval of z given a MONOTONIC

        tvec = self.tvec
        left = 0
        right = len(tvec)

        if search_val_i == tvec[-1]:
            zleft = len(tvec) - 2
            zright = len(tvec) - 1  # zero based indexing, last element is located at

        else:
            # binary search
            while (right > left + 1):
                mid = int(np.floor((left + right) / 2))
                if search_val_i < tvec[mid]:
                    right = mid

                else:
                    left = mid

            zleft = left
            zright = right

        return int(zleft), int(zright)

    def piecewise_func(self, t, coeffs):
        func_Pt = lambda t: {"line": np.array([1, t]),
                             'spline': np.array([1, t, t ** 2, t ** 3])}.get(self.interp_type)

        feval = np.dot(func_Pt(t), coeffs.flatten())

        return feval


if __name__ == '__main__':

    nx = 50
    x0 = np.linspace(0, 10, nx) + np.random.randn(nx) * 0
    y0 = np.sin(x0) + np.random.randn(nx) * 0
    ynoisy = y0 + np.random.randn(nx) * 0.5

    ## Prepare arc-length parametrization
    dx = np.diff(x0)
    dy = np.diff(y0)

    s0 = np.hypot(dx, dy).cumsum()
    s0 = np.hstack((0, s0))

    # Create autowareSpline
    interp_object_x = autowareSpline();

    # Define new coordinates
    nx_new = 100
    sfinal = s0[-1]
    snew = np.linspace(0, sfinal, nx_new)

    # Create interpolator
    yinterp = interp_object_x.interpolate(snew, s0, y0)

    plt.plot(s0, y0, 'r', lw=2, label='original curve')
    plt.plot(snew, yinterp, 'g-.', label='interpolated curve')

    plt.plot(s0, y0, 'ks', ms=3, label='original curve')
    plt.plot(snew, yinterp, 'go', ms=3, label='interpolated curve')
    plt.show()
