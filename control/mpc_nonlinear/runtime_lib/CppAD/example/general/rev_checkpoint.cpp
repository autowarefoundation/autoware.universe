/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin rev_checkpoint.cpp$$
$spell
    Checkpointing
    Taylor
$$

$section Reverse Mode General Case (Checkpointing): Example and Test$$

$head See Also$$
$cref/checkpoint/chkpoint_one/$$

$head Purpose$$
Break a large computation into pieces and only store values at the
interface of the pieces (this is much easier to do using $cref/checkpoint/chkpoint_one/$$).
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
$latex y^{(0)}$$ and $latex y^{(1)}$$.
$lnext
Free some, or all, of the memory corresponding to $latex F(x)$$.
$lnext
Use forward mode on $latex G(y)$$ to compute
$latex z^{(0)}$$ and $latex z^{(1)}$$
$lnext
Use reverse mode on $latex G(y)$$ to compute the derivative of
$latex h^{(1)}$$ with respect to
$latex y^{(0)}$$ and $latex y^{(1)}$$.
$lnext
Free all the memory corresponding to $latex G(y)$$.
$lnext
Use reverse mode on $latex F(x)$$ to compute the derivative of
$latex h^{(1)}$$ with respect to
$latex x^{(0)}$$ and $latex x^{(1)}$$.
$lend
This uses the following relations:
$latex \[
\begin{array}{rcl}
    \partial_{x(0)} h^{(1)} [ x^{(0)} , x^{(1)} ]
    & = &
    \partial_{y(0)} h^{(1)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(0)} y^{(0)} [ x^{(0)} , x^{(1)} ]
    \\
    & + &
    \partial_{y(1)} h^{(1)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(0)} y^{(1)} [ x^{(0)} , x^{(1)} ]
    \\
    \partial_{x(1)} h^{(1)} [ x^{(0)} , x^{(1)} ]
    & = &
    \partial_{y(0)} h^{(1)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(1)} y^{(0)} [ x^{(0)} , x^{(1)} ]
    \\
    & + &
    \partial_{y(1)} h^{(1)} [ y^{(0)} , y^{(1)} ]
    \partial_{x(1)} y^{(1)} [ x^{(0)} , x^{(1)} ]
\end{array}
\] $$
where $latex \partial_{x(0)}$$ denotes the partial with respect
to $latex x^{(0)}$$.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$


$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    template <class Vector>
    Vector F(const Vector& x)
    {   Vector y(2);
        y[0] = x[0] * x[1];
        y[1] = x[1] - x[0];
        return y;
    }
    template <class Vector>
    Vector G(const Vector& y)
    {   Vector z(2);
        z[0] = y[0] - y[1];
        z[1] = y[1] * y[0];
        return z;
    }
}

namespace {
    bool rev_checkpoint_case(bool free_all)
    {   bool ok = true;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

        using CppAD::AD;
        using CppAD::NearEqual;
        CppAD::ADFun<double> f, g, empty;

        // specify the Taylor coefficients for X(t)
        size_t n    = 2;
        CPPAD_TESTVECTOR(double) x0(n), x1(n);
        x0[0] = 1.; x0[1] = 2.;
        x1[0] = 3.; x1[1] = 4.;

        // record the function F(x)
        CPPAD_TESTVECTOR(AD<double>) X(n), Y(n);
        size_t i;
        for(i = 0; i < n; i++)
            X[i] = x0[i];
        CppAD::Independent(X);
        Y = F(X);
        f.Dependent(X, Y);

        // a fucntion object with an almost empty operation sequence
        CppAD::Independent(X);
        empty.Dependent(X, X);

        // compute the Taylor coefficients for Y(t)
        CPPAD_TESTVECTOR(double) y0(n), y1(n);
        y0 = f.Forward(0, x0);
        y1 = f.Forward(1, x1);
        if( free_all )
            f = empty;
        else
        {   // free all the Taylor coefficients stored in f
            f.capacity_order(0);
        }

        // record the function G(x)
        CPPAD_TESTVECTOR(AD<double>) Z(n);
        CppAD::Independent(Y);
        Z = G(Y);
        g.Dependent(Y, Z);

        // compute the Taylor coefficients for Z(t)
        CPPAD_TESTVECTOR(double) z0(n), z1(n);
        z0 = g.Forward(0, y0);
        z1 = g.Forward(1, y1);

        // check zero order Taylor coefficient for h^0 = z_0^0 + z_1^0
        double check = x0[0] * x0[1] * (1. - x0[0] + x0[1]) - x0[1] + x0[0];
        double h0    = z0[0] + z0[1];
        ok          &= NearEqual(h0, check, eps, eps);

        // check first order Taylor coefficient h^1
        check     = x0[0] * x0[1] * (- x1[0] + x1[1]) - x1[1] + x1[0];
        check    += x1[0] * x0[1] * (1. - x0[0] + x0[1]);
        check    += x0[0] * x1[1] * (1. - x0[0] + x0[1]);
        double h1 = z1[0] + z1[1];
        ok       &= NearEqual(h1, check, eps, eps);

        // compute the derivative with respect to y^0 and y^0 of h^1
        size_t p = 2;
        CPPAD_TESTVECTOR(double) w(n*p), dw(n*p);
        w[0*p+0] = 0.; // coefficient for z_0^0
        w[0*p+1] = 1.; // coefficient for z_0^1
        w[1*p+0] = 0.; // coefficient for z_1^0
        w[1*p+1] = 1.; // coefficient for z_1^1
        dw       = g.Reverse(p, w);

        // We are done using g, so we can free its memory.
        g = empty;
        // We need to use f next.
        if( free_all )
        {   // we must again record the operation sequence for F(x)
            CppAD::Independent(X);
            Y = F(X);
            f.Dependent(X, Y);
        }
        // now recompute the Taylor coefficients corresponding to F(x)
        // (we already know the result; i.e., y0 and y1).
        f.Forward(0, x0);
        f.Forward(1, x1);

        // compute the derivative with respect to x^0 and x^0 of
        //    h^1 = z_0^1 + z_1^1
        CPPAD_TESTVECTOR(double) dv(n*p);
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
        check  = 1. - x0[0] * x0[1];
        check += x0[1] * (1. - x0[0] + x0[1]);
        ok    &= NearEqual(dv[0*p+1], check, eps, eps);

        // check partial of h^1 w.r.t x^1_1
        check  = x0[0] * x0[1] - 1.;
        check += x0[0] * (1. - x0[0] + x0[1]);
        ok    &= NearEqual(dv[1*p+1], check, eps, eps);

        return ok;
    }
}
bool rev_checkpoint(void)
{   bool ok = true;
    ok     &= rev_checkpoint_case(true);
    ok     &= rev_checkpoint_case(false);
    return ok;
}

// END C++
