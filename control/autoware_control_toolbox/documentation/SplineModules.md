# SPLINE MODULES

The Autoware Control Toolbox includes spline, line fitting, and B-spline curve fitting and smoothers for computing curve
functions requiring second-order differentiation.

The numerical algorithms are derived for the smoothing splines by minimizing the Residual Sum of Squares (RSS),
which consists of two terms: fidelity and penalty terms in the form of;

$$
RSS(x) = \sum_{i=1}^N{(y_i - f(x_i))^2} + \lambda \int f^{\prime\prime}(t) dt
$$

where the first term formulates the fidelity part of the optimization whereas the second one with a smoothing factor
$\lambda$ penalizes the curve fit [2].

# Natural Cubic Spline (Piecewise) by Preconditioned Conjugate Gradients.

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