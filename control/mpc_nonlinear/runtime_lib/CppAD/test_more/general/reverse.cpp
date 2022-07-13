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
Old Reverse example now used just for valiadation testing
*/

# include <cppad/cppad.hpp>
namespace { // ----------------------------------------------------------

bool reverse_one(void)
{   bool ok = true;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) U(3);
    U[0] = 0.; U[1] = 1.; U[2] = 2.;
    Independent(U);

    // compute sum and product of elements in U
    AD<double> Sum  = 0.;
    AD<double> Prod = 1.;
    size_t i;
    for(i = 0; i < 3; i++)
    {   Sum  += U[i];
        Prod *= U[i];
    }

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) V(2);
    V[0] = Sum;
    V[1] = Prod;

    // V = f(U)
    ADFun<double> f(U, V);

    // Evaluate ( v[0] * f_0 + v[1] * f_1 )^(1) [ u0 ] ---------------
    size_t p  = 1;
    CPPAD_TESTVECTOR(double) v( f.Range() );
    CPPAD_TESTVECTOR(double) u0( f.Domain() );
    CPPAD_TESTVECTOR(double) r1( f.Domain() * p );

    v[0]  = 1.; v[1] = -1.;
    r1    = f.Reverse(p, v);

    // direct evaluation of gradients of components of f
    CPPAD_TESTVECTOR(double) g0(3), g1(3);
    u0[0] = Value(U[0]); u0[1] = Value(U[1]); u0[2] = Value(U[2]);
    g0[0] =          1.; g0[1] =          1.; g0[2] =          1.;
    g1[0] = u0[1]*u0[2]; g1[1] = u0[0]*u0[2]; g1[2] = u0[0]*u0[1];

    // compare values
    for(i = 0; i < 3; i++)
    {   ok &= NearEqual(r1[i] ,
            v[0] * g0[i] + v[1] * g1[i], eps99, eps99);
    }

    // -------------------------------------------------------------------

    // Define the function z(t, u0, u1) = f( u0 + u1 * t ) and evaluate
    // the first order Taylor coefficient column vector z(*, u0, u1)
    p = 1;
    CPPAD_TESTVECTOR(double) u1( f.Domain() );

    u1[0] = 2.; u1[1] = -1.; u1[2] = 3.;
    f.Forward(p, u1);

    // Evaluate the derivaties with respect to u0 of the functions
    // order 0: v[0] *      z_0 (0, u0, u1) + v[1] *      z_1 (0, u0, u1)
    // order 1: v[0] * d/dt z_0 (0, u0, u1) + v[1] * d/dt z_1 (0, u0, u1)
    p    = 2;
    CPPAD_TESTVECTOR(double) r2( f.Domain() * p );
    v[0] = -.5; v[1] = .5;
    r2   = f.Reverse(p, v);

    // check derivative of the zero order term
    for(i = 0; i < 3; i++)
    {   ok &= NearEqual(r2[p * i + 0] ,
            v[0] * g0[i] + v[1] * g1[i], eps99, eps99);
    }

    /*
    The j-th component of the first order term is
        d/dt z_j(0, u0, u1) = f_j^{(1)} (u0) * u1
    We use ei to denote the vector with its i-th component one and all
    the other components zero. The partial derivative of the j-th
    component of the first order term with respect u0[i] is
        ei * f_j^{(2)} ( u0 ) * u1
    */


    // direct evaluation of the Hessian f_1^{(2)} (u0)
    // (the Hessian f_0^{(2)} is identically zero)
    CPPAD_TESTVECTOR(double) H1(9);
    H1[0] =    0.; H1[1] = u0[2]; H1[2] = u0[1];
    H1[3] = u0[2]; H1[4] =    0.; H1[5] = u0[0];
    H1[6] = u0[1]; H1[7] = u0[0]; H1[8] =    0.;


    size_t j;
    for(i = 0; i < 3; i++)
    {   double sum = 0.;
        for(j = 0; j < 3; j++)
            sum += H1[i * 3 + j] * u1[j];

        // note term corresponding to v[0] is zero
        ok &= NearEqual(r2[p * i + 1], v[1] * sum, eps99, eps99);
    }

    return ok;
}

// define the template function reverse_any_cases<Vector> in empty namespace
template <class Vector>
bool reverse_any_cases(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n = 3;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;
    X[1] = 1.;
    X[2] = 2.;

    // declare independent variables and start recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = X[0] * X[1] * X[2];

    // create f : X -> Y and stop recording
    CppAD::ADFun<double> f(X, Y);

    // define W(t, u) = (u_0 + dx_0*t)*(u_1 + dx_1*t)*(u_2 + dx_2*t)
    // use zero order forward to evaluate W0(u) = W(0, u)
    Vector u(n), W0(m);
    u[0]    = 2.;
    u[1]    = 3.;
    u[2]    = 4.;
    W0      = f.Forward(0, u);
    double check;
    check   =  u[0]*u[1]*u[2];
    ok     &= NearEqual(W0[0] , check, eps99, eps99);

    // define W_t(t, u) = partial W(t, u) w.r.t t
    // W_t(t, u)  = (u_0 + dx_0*t)*(u_1 + dx_1*t)*dx_2
    //            + (u_0 + dx_0*t)*(u_2 + dx_2*t)*dx_1
    //            + (u_1 + dx_1*t)*(u_2 + dx_2*t)*dx_0
    // use first order forward mode to evaluate W1(u) = W_t(0, u)
    Vector dx(n), W1(m);
    dx[0] = .2;
    dx[1] = .3;
    dx[2] = .4;
    W1    = f.Forward(1, dx);
        check =  u[0]*u[1]*dx[2] + u[0]*u[2]*dx[1] + u[1]*u[2]*dx[0];
    ok   &= NearEqual(W1[0], check, eps99, eps99);

    // define W_tt (t, u) = partial W_t(t, u) w.r.t t
    // W_tt(t, u) = 2*(u_0 + dx_0*t)*dx_1*dx_2
    //            + 2*(u_1 + dx_1*t)*dx_0*dx_2
    //            + 2*(u_3 + dx_3*t)*dx_0*dx_1
    // use second order forward to evaluate W2(u) = 1/2 * W_tt(0, u)
    Vector ddx(n), W2(m);
    ddx[0] = ddx[1] = ddx[2] = 0.;
        W2     = f.Forward(2, ddx);
        check  =  u[0]*dx[1]*dx[2] + u[1]*dx[0]*dx[2] + u[2]*dx[0]*dx[1];
    ok    &= NearEqual(W2[0], check, eps99, eps99);

    // use third order reverse mode to evaluate derivatives
    size_t p = 3;
    Vector w(m), dw(n * p);
    w[0]   = 1.;
    dw     = f.Reverse(p, w);

    // check derivative of W0(u) w.r.t. u
    ok    &= NearEqual(dw[0*p+0], u[1]*u[2], eps99, eps99);
    ok    &= NearEqual(dw[1*p+0], u[0]*u[2], eps99, eps99);
    ok    &= NearEqual(dw[2*p+0], u[0]*u[1], eps99, eps99);

    // check derivative of W1(u) w.r.t. u
    ok    &= NearEqual(dw[0*p+1], u[1]*dx[2] + u[2]*dx[1], eps99, eps99);
    ok    &= NearEqual(dw[1*p+1], u[0]*dx[2] + u[2]*dx[0], eps99, eps99);
    ok    &= NearEqual(dw[2*p+1], u[0]*dx[1] + u[1]*dx[0], eps99, eps99);

    // check derivative of W2(u) w.r.t u
    ok    &= NearEqual(dw[0*p+2], dx[1]*dx[2], eps99, eps99);
    ok    &= NearEqual(dw[1*p+2], dx[0]*dx[2], eps99, eps99);
    ok    &= NearEqual(dw[2*p+2], dx[0]*dx[1], eps99, eps99);

    return ok;
}
/*
$comment rev_checkpoint.cpp$$
$spell
    Taylor
$$

$section Reverse Mode General Case: Example and Test$$

$index general, reverse example$$
$index reverse, general example$$
$index example, general reverse$$
$index test, general reverse$$

$index composition, example$$
$index example, composition$$
$index test, composition$$

$head Purpose$$
Break a derivative computation into pieces and only store values at the
interface of the pieces.
In actual applications, there may be many functions, but
for this example there are only two.
The functions
$latex F : \B{R}^2 \rightarrow \B{R}^2$$
and
$latex G : \B{R}^2 \rightarrow \B{R}^2$$
defined by
$latex \[
    F(x) = \left( \begin{array}{c} x_0 x_1   \\ x_1 - x_0 \end{array} \right)
    \; , \;
    G(y) = \left( \begin{array}{c} y_0 - y_1 \\ y_1  y_0   \end{array} \right)
\] $$
Another difference is that in actual applications,
the memory corresponding to function objects not currently being used
is sometimes returned to the system (see $cref/checkpoint/chkpoint_one/.cpp$$).

$head Processing Steps$$
We apply reverse mode to compute the derivative of
$latex H : \B{R}^2 \rightarrow \B{R}$$
is defined by
$latex \[
\begin{array}{rcl}
    H(x)
    & = & G_0 [ F(x) ] + G_1 [ F(x)  ]
    \\
    & = & x_0 x_1 - ( x_1 - x_0 ) + x_0 x_1 ( x_1 - x_0 )
    \\
    & = & x_0 x_1 ( 1 - x_0 + x_1 ) - x_1 + x_0
\end{array}
\] $$
Given the zero and first order Taylor coefficients
$latex x^{(0)} $$ and $latex x^{(1)}$$,
we use $latex X(t)$$, $latex Y(t)$$ and $latex Z(t)$$
for the corresponding functions; i.e.,
$latex \[
\begin{array}{rcl}
    X(t) & = & x^{(0)} + x^{(1)} t
    \\
    Y(t) & = & F[X(t)] = y^{(0)} + y^{(1)} t  + O(t^2)
    \\
    Z(t) & = & G \{ F [ X(t) ] \} = z^{(0)} + z^{(1)} t  + O(t^2)
    \\
    h^{(0)} & = & z^{(0)}_0 + z^{(0)}_1
    \\
    h^{(1)} & = & z^{(1)}_0 + z^{(1)}_1
\end{array}
\] $$
Here are the processing steps:
$list number$$
Use forward mode on $latex F(x)$$ to compute
$latex y^{(0)}$$ and $latex y^{(1)}$$
$lnext
Use forward mode on $latex G(y)$$ to compute
$latex z^{(0)}$$ and $latex z^{(1)}$$
$lnext
Use reverse mode on $latex G(y)$$ to compute the derivative of
$latex h^{(k)}$$ with respect to
$latex y^{(0)}$$ and $latex y^{(1)}$$.
$lnext
Use reverse mode on $latex F(x)$$ to compute the derivative of
$latex h^{(k)}$$ with respect to
$latex x^{(0)}$$ and $latex x^{(1)}$$.
$lend
This uses the following relations for $latex k = 0 , 1$$:
$latex \[
\begin{array}{rcl}
    \partial_{x(0)} h^{(k)} [ x^{(0)} , x^{(1)} ]
    & = &
    \partial_{y(0)} h^{(k)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(0)} y^{(0)} [ x^{(0)} , x^{(1)} ]
    \\
    & + &
    \partial_{y(1)} h^{(k)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(0)} y^{(1)} [ x^{(0)} , x^{(1)} ]
    \\
    \partial_{x(1)} h^{(k)} [ x^{(0)} , x^{(1)} ]
    & = &
    \partial_{y(0)} h^{(k)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(1)} y^{(0)} [ x^{(0)} , x^{(1)} ]
    \\
    & + &
    \partial_{y(1)} h^{(k)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(1)} y^{(1)} [ x^{(0)} , x^{(1)} ]
\end{array}
\] $$
where $latex \partial_{x(0)}$$ denotes the partial with respect
to $latex x^{(0)}$$.

$code
$comment%example/rev_checkpoint.cpp%0%// BEGIN C++%// END C++%1%$$
$$
$end
*/
template <class Vector>
Vector F_reverse_mul(const Vector& x)
{   Vector y(2);
    y[0] = x[0] * x[1];
    y[1] = x[1] - x[0];
    return y;
}
template <class Vector>
Vector G_reverse_mul(const Vector& y)
{   Vector z(2);
    z[0] = y[0] - y[1];
    z[1] = y[1] * y[0];
    return z;
}
bool reverse_mul(void)
{
    bool ok = true;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    using CppAD::AD;
    using CppAD::NearEqual;
    CppAD::ADFun<double> f, g;

    // Record the function F(x)
    size_t n    = 2;
    CPPAD_TESTVECTOR(AD<double>) X(n), Y(n);
    X[0] = X[1] = 0.;
    CppAD::Independent(X);
    Y = F_reverse_mul(X);
    f.Dependent(X, Y);

    // Record the function G(x)
    CPPAD_TESTVECTOR(AD<double>) Z(n);
    Y[0] = Y[1] = 0.;
    CppAD::Independent(Y);
    Z = G_reverse_mul(Y);
    g.Dependent(Y, Z);

    // argument and function values
    CPPAD_TESTVECTOR(double) x0(n), y0(n), z0(n);
    x0[0] = 1.;
    x0[1] = 2.;
    y0    = f.Forward(0, x0);
    z0    = g.Forward(0, y0);

    // check function value
    double check = x0[0] * x0[1] * (1. - x0[0] + x0[1]) - x0[1] + x0[0];
    double h0    = z0[0] + z0[1];
    ok          &= NearEqual(h0, check, eps, eps);

    // first order Taylor coefficients
    CPPAD_TESTVECTOR(double) x1(n), y1(n), z1(n);
    x1[0] = 3.;
    x1[1] = 4.;
    y1    = f.Forward(1, x1);
    z1    = g.Forward(1, y1);

    // check first order Taylor coefficients
    check     = x0[0] * x0[1] * (- x1[0] + x1[1]) - x1[1] + x1[0];
    check    += x1[0] * x0[1] * (1. - x0[0] + x0[1]);
    check    += x0[0] * x1[1] * (1. - x0[0] + x0[1]);
    double h1 = z1[0] + z1[1];
    ok       &= NearEqual(h1, check, eps, eps);

    // ----------------------------------------------------------------
    // dw^0 (y) = \partial_y^0 h^0 (y)
    // dw^1 (y) = \partial_y^1 h^0 (y)
    size_t p = 2;
    CPPAD_TESTVECTOR(double) w(n*p), dw(n*p);
    w[0*p+0] = 1.; // coefficient for z^0_0
    w[1*p+0] = 1.; // coefficient for z^0_1
    w[0*p+1] = 0.; // coefficient for z^1_0
    w[1*p+1] = 0.; // coefficient for z^1_1
    dw       = g.Reverse(p, w);

    // dv^0 = dw^0 * \partial_x^0 y^0 (x) + dw^1 * \partial_x^0 y^1 (x)
    // dv^1 = dw^0 * \partial_x^1 y^0 (x) + dw^1 * \partial_x^1 y^1 (x)
    CPPAD_TESTVECTOR(double) dv(n*p);
    dv   = f.Reverse(p, dw);

    // check partial of h^0 w.r.t x^0_0
    check  = x0[1] * (1. - x0[0] + x0[1]) + 1.;
    check -= x0[0] * x0[1];
    ok    &= NearEqual(dv[0*p+0], check, eps, eps);

    // check partial of h^0 w.r.t x^0_1
    check  = x0[0] * (1. - x0[0] + x0[1]) - 1.;
    check += x0[0] * x0[1];
    ok    &= NearEqual(dv[1*p+0], check, eps, eps);

    // check partial of h^0 w.r.t x^1_0 and x^1_1
    check  = 0.;
    ok    &= NearEqual(dv[0*p+1], check, eps, eps);
    ok    &= NearEqual(dv[1*p+1], check, eps, eps);

    // ----------------------------------------------------------------
    // dw^0 (y) = \partial_y^0 h^1 (y)
    // dw^1 (y) = \partial_y^1 h^1 (y)
    w[0*p+0] = 0.; // coefficient for z^0_0
    w[1*p+0] = 0.; // coefficient for z^0_1
    w[0*p+1] = 1.; // coefficient for z^1_0
    w[1*p+1] = 1.; // coefficient for z^1_1
    dw       = g.Reverse(p, w);

    // dv^0 = dw^0 * \partial_x^0 y^0 (x) + dw^1 * \partial_x^0 y^1 (x)
    // dv^1 = dw^0 * \partial_x^1 y^0 (x) + dw^1 * \partial_x^1 y^1 (x)
    dv   = f.Reverse(p, dw);

    // check partial of h^1 w.r.t x^0_0
    check  = x0[1] * (- x1[0] + x1[1]);
    check -= x1[0] * x0[1];
    check += x1[1] * (1. - x0[0] + x0[1]) - x0[0] * x1[1];
    ok    &= NearEqual(dv[0*p+0], check, eps, eps);

    // check partial of h^1 w.r.t x^0_1
    check  = x0[0] * (- x1[0] + x1[1]);
    check += x1[0] * (1. - x0[0] + x0[1]) + x1[0] * x0[1];
    check += x0[0] * x1[1];
    ok    &= NearEqual(dv[1*p+0], check, eps, eps);

    // check partial of h^1 w.r.t x^1_0
    // (by reverse mode identity is equal to partial h^0 w.r.t. x^0_0)
    check  = 1. - x0[0] * x0[1];
    check += x0[1] * (1. - x0[0] + x0[1]);
    ok    &= NearEqual(dv[0*p+1], check, eps, eps);

    // check partial of h^1 w.r.t x^1_1
    // (by reverse mode identity is equal to partial h^0 w.r.t. x^0_1)
    check  = x0[0] * x0[1] - 1.;
    check += x0[0] * (1. - x0[0] + x0[1]);
    ok    &= NearEqual(dv[1*p+1], check, eps, eps);

    return ok;
}
// ----------------------------------------------------------------------------
bool duplicate_dependent_var(void)
{   bool ok = true;
    using CppAD::AD;

    // f(x) = [ y_0, y_1 ] = [ x_0 * x_0 , x_0 * x_0 ]
    size_t nx = 1;
    size_t ny = 2;
    CPPAD_TESTVECTOR( AD<double> ) ax(nx), ay(ny);
    ax[0] = 1.0;
    CppAD::Independent(ax);
    ay[0] = ax[0] * ax[0];
    ay[1] = ay[0];
    CppAD::ADFun<double> f(ax, ay);
    //
    // Forward zero
    CPPAD_TESTVECTOR(double) x0(nx), x1(nx), yk(ny);
    x0[0] = 3.0;
    yk = f.Forward(0, x0);
    ok &= yk[0] == x0[0] * x0[0];
    ok &= yk[1] == yk[0];
    //
    // Forward one
    x1[0] = 1.0;
    yk = f.Forward(1, x1);
    ok &= yk[0] == 2.0 * x0[0];
    ok &= yk[1] == yk[0];
    //
    // Reverse two
    size_t q = 2; // number of orders
    CPPAD_TESTVECTOR(double) w(ny * q), dw(nx * q);
    for(size_t i = 0; i < ny; ++i)
    {   w[ i * q + 0 ] = 0.0; // derivative w.r.t. zero order coefficient
        w[ i * q + 1 ] = 1.0; // derivative w.r.t. first order coefficient
    }
    dw = f.Reverse(2, w);
    // derivative w.r.t zero order coefficients
    ok &= dw[nx * 0 + 0] == 4.0;
    // derivative w.r.t first order coefficients
    ok &= dw[nx * 0 + 1] == 4.0 * x0[0];
    //
    return ok;
}

} // End empty namespace

# include <vector>
# include <valarray>
bool reverse(void)
{   bool ok = true;
    ok &= reverse_one();
    ok &= reverse_mul();
    ok &= duplicate_dependent_var();

    ok &= reverse_any_cases< CppAD::vector  <double> >();
    ok &= reverse_any_cases< std::vector    <double> >();
    ok &= reverse_any_cases< std::valarray  <double> >();
    return ok;
}
