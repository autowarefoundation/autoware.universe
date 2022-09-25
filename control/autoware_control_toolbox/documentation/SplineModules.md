# SPLINE MODULES

The Autoware Control Toolbox provides spline, line fitting and B-spline curve smoothers which can be used to compute
curve functions that require second order differentiation.

\begin{equation}
\operatorname{RSS}(f, \lambda)=\sum_{i=1}^N\left\{y_i-f\left(x_i\right)\right\}^2+\lambda \int\left\{f^{\prime \prime}(
t)\right\}^2 d t
\end{equation}

### References

1. Ruppert, D., Wand, M.P. and Carroll, R.J., 2003. Semiparametric regression (No. 12). Cambridge university press.
2. Hastie, T., Tibshirani, R., Friedman, J.H. and Friedman, J.H., 2009. The elements of statistical learning: data
   mining, inference, and prediction (Vol. 2, pp. 1-758). New York: springer.