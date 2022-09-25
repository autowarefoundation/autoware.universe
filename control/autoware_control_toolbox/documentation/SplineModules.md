# SPLINE MODULES

The Autoware Control Toolbox includes spline, line fitting, and B-spline curve smoothers for computing curve functions
requiring second-order differentiation.

Smoothing splines are calculated by minimizing the Residual Sum of Squares (RSS), which consists of two terms: fidelity
and penalty terms in the form of;

$$
RSS(x) = \sum_{i=1}^N{(y_i - f(x_i))^2} + \lambda \int f^{\prime\prime}(t) dt
$$

where the first term formulates the fidelity part of the optimization whereas the second one with a smoothing factor
$\lambda$ penalizes the curve fit [2].

### References

1. Ruppert, D., Wand, M.P. and Carroll, R.J., 2003. Semiparametric regression (No. 12). Cambridge university press.
2. Hastie, T., Tibshirani, R., Friedman, J.H. and Friedman, J.H., 2009. The elements of statistical learning: data
   mining, inference, and prediction (Vol. 2, pp. 1-758). New York: springer.