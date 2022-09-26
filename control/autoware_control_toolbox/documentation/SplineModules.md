# SPLINE MODULES

## BSpline Smoothers

The Autoware Control Toolbox includes spline, line fitting, and B-spline curve fitting and smoothers for computing curve
functions requiring second-order differentiation.

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

where on $\hat{y}$ represents the interpolated and smoothed signal, whereas $y$ on the left-hand side is the original
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

## Natural Cubic Spline (Piecewise) by Preconditioned Conjugate Gradients.

The spline modules provide a piecewise cubic (and with a linear option) class which can be used to fit a curve or
line to the given data piecewise. The implementation depends on the Eigen library, the matrix version of
the spline method available in Autoware based on the C++ STL vectors. The following code snippets shows how to
construct and use the interpolators.

```c++

// Generate a sinusoidal signal with time parametrization.
size_t const num_of_points = 100;  // number of points in the signal.

double const &omega = 0.3;  // sin-wave speed.

// Generate x.
std::vector<double> tbase = ns_utils::linspace(0.0, 10.0, num_of_points);

// Generate y = sin(t).
std::vector<double> ybase;
std::transform(
tbase.cbegin(), tbase.cend(), std::back_inserter(ybase),[&omega](auto const &t)
{
return sin(omega * t); });

// Create new interpolating coordinates
size_t const &new_num_of_points = 50;
auto tnew = ns_utils::linspace(tbase[0], tbase.back(), new_num_of_points);

std::vector<double> yvalidation;
std::transform(
tnew.cbegin(), tnew.cend(), std::back_inserter(yvalidation),[&omega](auto const &t)
{
return sin(omega * t); });

// Create a spline object from the x
// Default spline type is 'spline'. We can choose 'line' as an additional implementation.

size_t const &interpolating_type = 3;  // 3 for spline, 1 for line.
auto interpolating_spline_aw = ns_splines::InterpolatingSplinePCG(interpolating_type);

/**
 * Interpolate and store the solutions in yinterp. The interpolator does not keep the computed spline coefficients.
 * This construction is memoryless.
 * */
std::vector<double> yinterp;
auto is_interpolated = interpolating_spline_aw.Interpolate(tbase, ybase, tnew, yinterp);
ns_utils::print("Is interpolated :", is_interpolated);

```

### References

1. Ruppert, D., Wand, M.P. and Carroll, R.J., 2003. Semiparametric regression (No. 12). Cambridge university press.
2. Hastie, T., Tibshirani, R., Friedman, J.H. and Friedman, J.H., 2009. The elements of statistical learning: data
   mining, inference, and prediction (Vol. 2, pp. 1-758). New York: springer.
3. https://en.wikipedia.org/wiki/Tikhonov_regularization