/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

/*
$begin change_param.cpp$$
$spell
    Jacobian
$$

$section Computing a Jacobian With Constants that Change$$

$head Purpose$$
In this example we use two levels of taping so that a derivative
can have constant parameters that can be changed. To be specific,
we consider the function $latex f : \B{R}^2 \rightarrow \B{R}^2$$
$latex \[
f(x) = p \left( \begin{array}{c}
    \sin( x_0 ) \\
    \sin( x_1 )
\end{array} \right)
\]$$
were $latex p \in \B{R}$$ is a parameter.
The Jacobian of this function is
$latex \[
g(x,p) = p \left( \begin{array}{cc}
    \cos( x_0 ) & 0 \\
    0           & \cos( x_1 )
\end{array} \right)
\] $$
In this example we use two levels of AD to avoid computing
the partial of $latex f(x)$$ with respect to $latex p$$,
but still allow for the evaluation of $latex g(x, p)$$
at different values of $latex p$$.

$end

*/

bool change_param(void)
{   bool ok = true;                     // initialize test result

    typedef CppAD::AD<double> a1type;   // for first level of taping
    typedef CppAD::AD<a1type> a2type;  // for second level of taping

    size_t nu = 3;       // number components in u
    size_t nx = 2;       // number components in x
    size_t ny = 2;       // num components in f(x)
    size_t nJ = ny * nx; // number components in Jacobian of f(x)

    // temporary indices
    size_t j;

    // declare first level of independent variables
    // (Start taping now so can record dependency of a1f on a1p.)
    CPPAD_TESTVECTOR(a1type) a1u(nu);
    for(j = 0; j < nu; j++)
        a1u[j] = 0.;
    CppAD::Independent(a1u);

    // parameter in computation of Jacobian
    a1type a1p = a1u[2];

    // declare second level of independent variables
    CPPAD_TESTVECTOR(a2type) a2x(nx);
    for(j = 0; j < nx; j++)
        a2x[j] = 0.;
    CppAD::Independent(a2x);

    // compute dependent variables at second level
    CPPAD_TESTVECTOR(a2type) a2y(ny);
    a2y[0] = sin( a2x[0] ) * a1p;
    a2y[1] = sin( a2x[1] ) * a1p;

    // declare function object that computes values at the first level
    // (make sure we do not run zero order forward during constructor)
    CppAD::ADFun<a1type> a1f;
    a1f.Dependent(a2x, a2y);

    // compute the Jacobian of a1f at a1u[0], a1u[1]
    CPPAD_TESTVECTOR(a1type) a1x(nx);
    a1x[0] = a1u[0];
    a1x[1] = a1u[1];
    CPPAD_TESTVECTOR(a1type) a1J(nJ);
    a1J = a1f.Jacobian( a1x );

    // declare function object that maps u = (x, p) to Jacobian of f
    // (make sure we do not run zero order forward during constructor)
    CppAD::ADFun<double> g;
    g.Dependent(a1u, a1J);

    // remove extra variables used during the reconding of a1f,
    // but not needed any more.
    g.optimize();

    // compute the Jacobian of f using zero order forward
    // sweep with double values
    CPPAD_TESTVECTOR(double) J(nJ), u(nu);
    for(j = 0; j < nu; j++)
        u[j] = double(j+1);
    J = g.Forward(0, u);

    // accuracy for tests
    double eps = 100. * CppAD::numeric_limits<double>::epsilon();

    // y[0] = sin( x[0] ) * p
    // y[1] = sin( x[1] ) * p
    CPPAD_TESTVECTOR(double) x(nx);
    x[0]      = u[0];
    x[1]      = u[1];
    double p  = u[2];

    // J[0] = partial y[0] w.r.t x[0] = cos( x[0] ) * p
    double check = cos( x[0] ) * p;
    ok   &= fabs( check - J[0] ) <= eps;

    // J[1] = partial y[0] w.r.t x[1] = 0.;
    check = 0.;
    ok   &= fabs( check - J[1] ) <= eps;

    // J[2] = partial y[1] w.r.t. x[0] = 0.
    check = 0.;
    ok   &= fabs( check - J[2] ) <= eps;

    // J[3] = partial y[1] w.r.t x[1] = cos( x[1] ) * p
    check = cos( x[1] ) * p;
    ok   &= fabs( check - J[3] ) <= eps;

    return ok;
}
// END PROGRAM
