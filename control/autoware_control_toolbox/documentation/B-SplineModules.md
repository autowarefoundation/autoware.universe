## BSpline Smoothers

The Autoware Control Toolbox includes B-spline curve fitting and smoothers modules. Unlike the natural spline or
piece-wise curve fitting methods available in the interpolation package, the B-spline modules fit a global curve
smoothing the given data points (the fitted curve does not pass through the data points). The B-spline module can
compute the first and second derivative of the requested curve polynomial, which can be used to compute a yaw or
curvature trajectory along the curve.

The numerical algorithms are derived for the smoothing splines by minimizing the Residual Sum of Squares (RSS),
which consists of two terms: fidelity and penalty terms in the form of;

$$
RSS(x) = \sum_{i=1}^N{(y_i - f(x_i))^2} + \lambda \int f^{\prime\prime}(t) dt
$$

where the first term formulates the fidelity part of the optimization whereas the second one with a smoothing factor
$\lambda$ penalizes the curve fit [2].

The result of the optimization the regression equation can be written as

$$
\hat{\mathbf{y}}=\mathbf{X}\left(\mathbf{X}^{\top} \mathbf{X}+\lambda^{2 p} \mathbf{D}\right)^{-1} \mathbf{X}^{\top}
\mathbf{y}
$$

where on $\hat{y}$ represents the interpolated and smoothed signal, whereas $y$ on the right-hand side is the original
signal to be smoothed. Since we use Tikhonov regularization [3], $D$ in the equation is an idendity matrix. The input
signal $y$ can have multiple columns. The regressor

$$
\mathbf{X}\left(\mathbf{X}^{\top} \mathbf{X}+\lambda^{2 p} \mathbf{D}\right)^{-1} \mathbf{X}^{\top}
$$

needs to be computed only once as we used only implicit parametrization $y = y(t)$.

In the following code snippet we show that how to use the B-Spline smoother to compute curvature given $x-y$
coordinates of a path.

```c++
// Generate a noisy sinusoidal signal with arc-length parametrization. This is our test signal.
const size_t Nin = 120;  // input dimension
const size_t Nout = 80; // output dimension - down-sampled

// Assume we have a noisy signal.
double const &add_noise = 1;

std::random_device rd;
std::default_random_engine generator{ rd() };
std::normal_distribution<double> distribution(0.0, 0.3);

// Generate x coordinate.
double kx = 8;
std::vector<double> xvec = ns_utils::linspace<double>(0.0, kx, Nin);

// To Eigen matrix
Eigen::MatrixXd xe;
xe = Eigen::Map<Eigen::Matrix<double, Nin, 1 >>(xvec.data());

// Generate y coordinate
Eigen::MatrixXd ye(xe.unaryExpr([&](auto x)
{
return cy * sin(x) + distribution(generator) * add_noise; }));

// for interpolating another signal with the same regressor.
Eigen::MatrixXd ze(xe.unaryExpr([&](auto x)
{
return 2 * cos(x) - 3 * sin(x) + distribution(generator) * add_noise; }));


// Evaluate Interpolator
Eigen::MatrixXd yinterp(ye.rows(), ye.cols());

// Create a new smoothing spline.
double know_number_ratio = 0.3;  // percentage ratio of the number of know points to the input size.
bool compute_derivatives{ true };

// Create the smoother.
ns_splines::BSplineInterpolatorTemplated<Nin, Nout> interpolating_bspline(know_number_ratio, compute_derivatives);

// Store the interpolated data
interpolating_bspline.InterpolateImplicitCoordinates(ye, yinterp);

// Evaluate another variable using the same Interpolator
Eigen::MatrixXd zinterp(ye.rows(), ye.cols());
interpolating_bspline.InterpolateImplicitCoordinates(ze, zinterp);

// Computing curvature
// Get xdot, ydot
Eigen::MatrixXd rdot_interp(Nout, 2); // [xdot, ydot]

// Get xddot, yddot
Eigen::MatrixXd rddot_interp(Nout, 2); // [xddot, yddot]

auto xy_data = ns_eigen_utils::hstack<double>(xe, ye);

interpolating_bspline.getFirstDerivative(xy_data, rdot_interp);
interpolating_bspline.getSecondDerivative(xy_data, rddot_interp);

// Curvature from the B-spline
Eigen::MatrixXd curvature_bspline_smoother;
curvature_bspline_smoother = ns_eigen_utils::Curvature(rdot_interp, rddot_interp);
```

### References

1. Ruppert, D., Wand, M.P. and Carroll, R.J., 2003. Semiparametric regression (No. 12). Cambridge university press.
2. Hastie, T., Tibshirani, R., Friedman, J.H. and Friedman, J.H., 2009. The elements of statistical learning: data
   mining, inference, and prediction (Vol. 2, pp. 1-758). New York: springer.
3. <https://en.wikipedia.org/wiki/Tikhonov_regularization>
