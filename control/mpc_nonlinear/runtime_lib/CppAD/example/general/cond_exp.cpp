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
$begin cond_exp.cpp$$
$spell
    Cpp
    cstddef
    CondExp
$$

$section Conditional Expressions: Example and Test$$

$head See Also$$
$cref optimize_conditional_skip.cpp$$


$head Description$$
Use $code CondExp$$ to compute
$latex \[
    f(x) = \sum_{j=0}^{m-1} x_j \log( x_j )
\] $$
and its derivative at various argument values
( where $latex x_j \geq 0$$ )
with out having to re-tape; i.e.,
using only one $cref ADFun$$ object.
Note that $latex x_j \log ( x_j ) \rightarrow 0 $$
as $latex x_j \downarrow 0$$ and
we need to handle the case $latex x_j = 0$$
in a special way to avoid returning zero times minus infinity.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <limits>

bool CondExp(void)
{   bool ok = true;

    using CppAD::isnan;
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::log;
    double eps  = 100. * CppAD::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 5;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    size_t j;
    for(j = 0; j < n; j++)
        ax[j] = 1.;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    AD<double> asum  = 0.;
    AD<double> azero = 0.;
    for(j = 0; j < n; j++)
    {   // if x_j > 0, add x_j * log( x_j ) to the sum
        asum += CppAD::CondExpGt(ax[j], azero, ax[j] * log(ax[j]), azero);
    }

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = asum;

    // create f: x -> ay and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // vectors for arguments to the function object f
    CPPAD_TESTVECTOR(double) x(n);   // argument values
    CPPAD_TESTVECTOR(double) y(m);   // function values
    CPPAD_TESTVECTOR(double) w(m);   // function weights
    CPPAD_TESTVECTOR(double) dw(n);  // derivative of weighted function

    // a case where x[j] > 0 for all j
    double check  = 0.;
    for(j = 0; j < n; j++)
    {   x[j]   = double(j + 1);
        check += x[j] * log( x[j] );
    }

    // function value
    y  = f.Forward(0, x);
    ok &= NearEqual(y[0], check, eps, eps);

    // compute derivative of y[0]
    w[0] = 1.;
    dw   = f.Reverse(1, w);
    for(j = 0; j < n; j++)
        ok &= NearEqual(dw[j], log(x[j]) + 1., eps, eps);

    // a case where x[3] is equal to zero
    check -= x[3] * log( x[3] );
    x[3]   = 0.;
    ok &= std::isnan( x[3] * log( x[3] ) );

    // function value
    y   = f.Forward(0, x);
    ok &= NearEqual(y[0], check, eps, eps);

    // check derivative of y[0]
    w[0] = 1.;
    dw   = f.Reverse(1, w);
    for(j = 0; j < n; j++)
    {   if( x[j] > 0 )
            ok &= NearEqual(dw[j], log(x[j]) + 1., eps, eps);
        else
            ok &= NearEqual(dw[j], 0.0, eps, eps);
    }

    return ok;
}
// END C++
