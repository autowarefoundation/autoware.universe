import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag

np.printoptions(precision=3, suppress='true')


class Interp1d():
    def __init__(self, data, parameter_vec_ts=None, interpolation_type='spline'):
        '''

        :param data: data is 1D vector
        :param parameter_ts: curve length or time parametrization,
        '''

        self.data = data  # data points to be interpolated
        self.data_shifted = self.shift_data()  # create intervals x0-x1
        self.nx = len(data)  # We split data into two part by shifting
        self.ni = self.nx - 1  # number of intervals

        self.interp_type = interpolation_type  # degree of polynomial - ['linear', 'spline']

        '''
            In all cases of the parametrization, each piece of the polynomial time or arc length is parametrized 
            between 0 and 1. 
        '''
        if parameter_vec_ts is not None:
            """
                We need to normalize each interval 
            """
            self.tvec = parameter_vec_ts

        else:
            '''
                time vector is a collection of indices with an increment 1. 
            '''
            self.tvec = np.arange(0, self.nx)

        '''
            Prepare and solve the system 
            Ax = b (A=A(t) polynomials, x is the theta =[a, b*t, c*t**2,  dt**3]  parameters  
            Cx = d 
            
            Once A and C are computed, we can interpolate any time vector on it. 
        '''

        A, C = self.get_A_and_C()  # doing explicitly to show the algorithm flow.
        self.A = A
        self.C = C

        self.polynomials = self.cls_solve()

    def shift_data(self):
        x = self.data
        shifted = np.hstack((x[:-1], x[1:])).reshape(-1, 1)

        return shifted

    def get_A_and_C(self):
        '''
            A is the polynomial matrix
            C is the constraint matrix
        :return: returns A and C matrices
        '''

        primitive_spline = np.array([[1, 1, 1, 1, -1, 0, 0, 0],
                                     [0, 1, 2, 3, 0, -1, 0, 0],
                                     [0, 0, 2, 6, 0, 0, -2, 0]])

        primitive_line = np.array([[1, 1, -1, 0]])

        # I put lambda function in case we use different t or s values. But for time being it is sufficient to
        # parametrize all the intervals between [0, and 1]

        f_tprimitive = lambda t: {"line": [1, t], "spline": [1, t, t ** 2, t ** 3]}.get(self.interp_type)
        cprimitive = {"line": primitive_line, "spline": primitive_spline}.get(self.interp_type)

        ## Construct A matrix
        '''
            We have two data points for each polynomial, therefore we have two sets of equations. 
            at t=0 --- > A0 x = b0 # at the start of the polynomial the data are known
            at t=1 --- > A1 x = b1 # at the end of the polynomal the data are known. 
        '''

        # block_diag(*[tprimitive_vec] * mx_1)
        diags_A0 = [f_tprimitive(0) for k in range(self.ni)]
        diags_A1 = [f_tprimitive(1) for k in range(self.ni)]

        A0 = block_diag(*diags_A0)
        A1 = block_diag(*diags_A1)

        A = np.vstack((A0, A1))

        ## Construct the constraint matrix
        cm, cn = cprimitive.shape
        ind_half = int(cn / 2)

        Blockf0 = cprimitive[:, 0:ind_half]
        Blockf1 = cprimitive[:, ind_half:]

        eyef0 = np.eye(self.ni, k=0)  # Good notes C++ notes
        eyef0 = eyef0[:-1, :]

        eyef1 = np.eye(self.ni, k=0)
        eyef1 = eyef1[1:, :]

        C0 = np.kron(eyef0, Blockf0)
        C1 = np.kron(eyef1, Blockf1)

        C = C0 + C1

        # self.C = C

        return A, C

    def cls_solve(self):
        '''
            Solution to the system of equations:
            Ax = b --> polynomial fit, x is the parameters
            Cx = d --> constraints

        :return:
        '''

        m, n = self.A.shape  # Ax = b, Cx = d Boyd pp 349 Intro to applied linear algebra
        p, n = self.C.shape
        Q, R = np.linalg.qr(np.vstack([self.A, self.C]))

        b = self.data_shifted  # set shifted tata [X(t=0), X(t=1)]
        d = np.zeros((p, 1))

        Q1 = Q[:m, :]
        Q2 = Q[m:m + p + 1, :]

        # Second QR factorization for Q2
        Qtil, Rtil = np.linalg.qr(Q2.T)
        '''

        Rtilde * w  = 2 Qtilde * Q1.transpose * b = 2 Rtilde_inv_transpose - 2*Rtilde_inv_transpose * d    
        Rtilde * w = b2
        '''

        b2 = 2 * Qtil.T @ (Q1.T @ b) - 2 * np.linalg.inv(Rtil.T) @ d
        w = np.linalg.pinv(Rtil) @ b2

        # Substitute we and solve another equation
        xhat = np.linalg.pinv(R) @ (Q1.T @ b - Q2.T @ w / 2)

        return xhat

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

    def __call__(self, parameter_vec_ts):
        '''
        We use binary tree search.
        :param parameter_vec_ts: vector of parameters at which the splines are evaluated.
        :return:
        '''

        ## Number of coefficients
        ns = {'line': 2, 'spline': 4}.get(self.interp_type)
        tvec = parameter_vec_ts
        theta = self.polynomials  # matrix of [a, b*t, c*t**2, d*t*3] polynomial coefficients or [a + b*t] for line.

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
            indstart = ind_left * ns
            indend = ind_right * ns

            # Pick the coeffs at this interval
            piece_coeffs = theta[indstart:indend]

            ## Compute the dt of this ti; 
            dt = ti - self.tvec[ind_left]  # ti is always greater than ind_left.
            dt = dt / (self.tvec[ind_right] - self.tvec[ind_left])  # normalize

            assert dt <= 1 and dt >= 0, "dt must be in [0, 1]"

            xval = self.piecewise_func(dt, piece_coeffs)

            ## append to xinterp
            xinterp.append(xval)

        return np.array(xinterp)


if __name__ == "__main__":
    ## Generate some data

    nx = 20
    x = np.linspace(0, 10, nx) + np.random.randn(nx) * 0.1
    y = np.sin(x) + np.random.randn(nx) * 0.2

    ## Prepare arc-length parametrization
    dx = np.diff(x)
    dy = np.diff(y)

    s0 = np.hypot(dx, dy).cumsum()
    s0 = np.hstack((0, s0))

    '''
        A theta = b -- > b is data matrix, theta is the coefficients, A is A(t) functions of paremetrization
        C is the constraint matrix. 
        
        Ax = b
        Cx = d 
    '''

    ## Interpolate new data points
    '''
        we have nx data point and nx-1 intervals
    '''
    sfinal = s0[-1]
    snew = np.linspace(0, sfinal, 100)

    ## ----------- New Implementation -----------------------------###
    interp_object_x = Interp1d(data=x, parameter_vec_ts=s0, interpolation_type='spline')
    xinterp = interp_object_x(snew)

    interp_object_y = Interp1d(data=y, parameter_vec_ts=s0, interpolation_type='spline')
    yinterp = interp_object_y(snew)

    plt.plot(x, y, 'r-.', label='original curve')
    plt.plot(xinterp, yinterp, alpha=0.6, label='interpolated curve')
    plt.show()
 
    aa = 1
