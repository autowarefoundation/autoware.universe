# SPLINE MODULES

The Autoware Control Toolbox provides spline, line fitting and B-spline curve smoothers which can be used to compute
curve functions that require second order differentiation.

$$
RSS(x) = \sum{(y_i - f(x_i))^2} + \lambda \int f^{\prime\prime}(t) dt
$$

### References

1. Ruppert, D., Wand, M.P. and Carroll, R.J., 2003. Semiparametric regression (No. 12). Cambridge university press.
2. Hastie, T., Tibshirani, R., Friedman, J.H. and Friedman, J.H., 2009. The elements of statistical learning: data
   mining, inference, and prediction (Vol. 2, pp. 1-758). New York: springer.