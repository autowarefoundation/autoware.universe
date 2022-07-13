/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin pow_nan.cpp$$
$spell
$$

$section pow: Nan in Result of Pow Function: Example and Test$$

$head Purpose$$
The $cref%pow(x, y)%pow%$$ function will work when $latex x < 0$$ and
$latex y$$  is a parameter. It will often generate nan or infinity when
$latex x < 0$$ and one tries to compute a derivatives
(even if $latex y$$ is a positive integer).
This is because the derivative of the log is $latex 1 / x$$
and the power function uses the representation
$latex \[
    \R{pow}(x, y) = \exp [ y \cdot \log(x) ]
\] $$

$head Problem$$
There is a problem with this representation when $latex y$$ is a parameter
and $latex x = 0$$. For example,
when $latex x = 0$$ and $latex y = 1$$, it returns zero for the derivative,
but the actual derivative w.r.t $latex x$$ is one.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include <cmath>

bool pow_nan(void)
{   bool ok = true;

    using std::cout;
    using CppAD::AD;
    using CppAD::vector;
    //
    vector<double>       x(2), y(2), dx(2), dy(2), ddx(2), ddy(2);
    vector< AD<double> > ax(2), ay(2);
    //
    // variable vector
    ax[0] = x[0]  = -1.0;
    ax[1] = x[1] = 2.0;
    //
    CppAD::Independent(ax);
    //
    ay[0] = pow( ax[0], ax[1] );
    ay[1] = pow( ax[0], 2.0 );
    //
    CppAD::ADFun<double> f(ax, ay);
    //
    // check_for_nan is set false so it does not generate an assert
    // when compiling with debugging
    f.check_for_nan(false);
    //
    // Zero order forward does not generate nan
    y  = f.Forward(0, x);
    ok &= y[0] == 1.0;
    ok &= y[1] == 1.0;
    //
    // First order forward generates a nan
    dx[0] = 1.0;
    dx[1] = 1.0;
    dy = f.Forward(1, dx);
    ok &= std::isnan( dy[0] );
    ok &= dy[1] == -2.0;
    //
    // Second order Taylor coefficient is 1/2 times second derivative
    ddx[0] = 0.0;
    ddx[1] = 0.0;
    ddy = f.Forward(2, ddx);
    ok &= std::isnan( ddy[0] );
    ok &= ddy[1] == 1.0;
    //
    return ok;
}
// END C++
