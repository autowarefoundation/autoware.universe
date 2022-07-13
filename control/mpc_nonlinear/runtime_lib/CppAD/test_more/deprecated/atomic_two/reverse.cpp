/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_two_reverse.cpp$$
$spell
    Jacobian
$$

$section Atomic Reverse: Example and Test$$

$head Purpose$$
This example demonstrates reverse mode derivative calculation
using an atomic operation.

$head function$$
For this example, the atomic function
$latex f : \B{R}^3 \rightarrow \B{R}^2$$ is defined by
$latex \[
f(x) = \left( \begin{array}{c}
    x_2 * x_2 \\
    x_0 * x_1
\end{array} \right)
\] $$
The corresponding Jacobian is
$latex \[
f^{(1)} (x) = \left( \begin{array}{ccc}
  0  &   0 & 2 x_2 \\
x_1  & x_0 & 0
\end{array} \right)
\] $$
The Hessians of the component functions are
$latex \[
f_0^{(2)} ( x ) = \left( \begin{array}{ccc}
    0 & 0 & 0  \\
    0 & 0 & 0  \\
    0 & 0 & 2
\end{array} \right)
\W{,}
f_1^{(2)} ( x ) = \left( \begin{array}{ccc}
    0 & 1 & 0 \\
    1 & 0 & 0 \\
    0 & 0 & 0
\end{array} \right)
\] $$

$nospell

$head Start  Class Definition$$
$srccode%cpp% */
# include <cppad/cppad.hpp>
namespace {          // isolate items below to this file
using CppAD::vector; // abbreviate as vector
//
class atomic_reverse : public CppAD::atomic_base<double> {
/* %$$
$head Constructor $$
$srccode%cpp% */
public:
    // constructor (could use const char* for name)
    atomic_reverse(const std::string& name) :
    // this example does not use sparsity patterns
    CppAD::atomic_base<double>(name)
    { }
private:
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    virtual bool forward(
        size_t                    p ,
        size_t                    q ,
        const vector<bool>&      vx ,
        vector<bool>&            vy ,
        const vector<double>&    tx ,
        vector<double>&          ty
    )
    {
        size_t q1 = q + 1;
# ifndef NDEBUG
        size_t n = tx.size() / q1;
        size_t m = ty.size() / q1;
# endif
        assert( n == 3 );
        assert( m == 2 );
        assert( p <= q );

        // this example only implements up to first order forward mode
        bool ok = q <= 1;
        if( ! ok )
            return ok;

        // check for defining variable information
        // This case must always be implemented
        if( vx.size() > 0 )
        {   vy[0] = vx[2];
            vy[1] = vx[0] || vx[1];
        }
        // ------------------------------------------------------------------
        // Zero forward mode.
        // This case must always be implemented
        // f(x) = [ x_2 * x_2 ]
        //        [ x_0 * x_1 ]
        // y^0  = f( x^0 )
        if( p <= 0 )
        {   // y_0^0 = x_2^0 * x_2^0
            ty[0 * q1 + 0] = tx[2 * q1 + 0] * tx[2 * q1 + 0];
            // y_1^0 = x_0^0 * x_1^0
            ty[1 * q1 + 0] = tx[0 * q1 + 0] * tx[1 * q1 + 0];
        }
        if( q <= 0 )
            return ok;
        // ------------------------------------------------------------------
        // First order one forward mode.
        // This case is needed if first order forward mode is used.
        // f'(x) = [   0,   0, 2 * x_2 ]
        //         [ x_1, x_0,       0 ]
        // y^1 =  f'(x^0) * x^1
        if( p <= 1 )
        {   // y_0^1 = 2 * x_2^0 * x_2^1
            ty[0 * q1 + 1] = 2.0 * tx[2 * q1 + 0] * tx[2 * q1 + 1];

            // y_1^1 = x_1^0 * x_0^1 + x_0^0 * x_1^1
            ty[1 * q1 + 1]  = tx[1 * q1 + 0] * tx[0 * q1 + 1];
            ty[1 * q1 + 1] += tx[0 * q1 + 0] * tx[1 * q1 + 1];
        }
        return ok;
    }
/* %$$
$head reverse$$
$srccode%cpp% */
    // reverse mode routine called by CppAD
    virtual bool reverse(
        size_t                   q ,
        const vector<double>&    tx ,
        const vector<double>&    ty ,
        vector<double>&          px ,
        const vector<double>&    py
    )
    {
        size_t q1 = q + 1;
        size_t n = tx.size() / q1;
# ifndef NDEBUG
        size_t m = ty.size() / q1;
# endif
        assert( n == 3 );
        assert( m == 2 );

        // this example only implements up to second order reverse mode
        bool ok = q1 <= 2;
        if( ! ok )
            return ok;
        //
        // initalize summation as zero
        for(size_t j = 0; j < n; j++)
            for(size_t k = 0; k < q1; k++)
                px[j * q1 + k] = 0.0;
        //
        if( q1 == 2 )
        {   // --------------------------------------------------------------
            // Second order reverse first compute partials of first order
            // We use the notation pf_ij^k for partial of F_i^1 w.r.t. x_j^k
            //
            // y_0^1    = 2 * x_2^0 * x_2^1
            // pf_02^0  = 2 * x_2^1
            // pf_02^1  = 2 * x_2^0
            //
            // y_1^1    = x_1^0 * x_0^1 + x_0^0 * x_1^1
            // pf_10^0  = x_1^1
            // pf_11^0  = x_0^1
            // pf_10^1  = x_1^0
            // pf_11^1  = x_0^0
            //
            // px_0^0 += py_0^1 * pf_00^0 + py_1^1 * pf_10^0
            //        += py_1^1 * x_1^1
            px[0 * q1 + 0] += py[1 * q1 + 1] * tx[1 * q1 + 1];
            //
            // px_0^1 += py_0^1 * pf_00^1 + py_1^1 * pf_10^1
            //        += py_1^1 * x_1^0
            px[0 * q1 + 1] += py[1 * q1 + 1] * tx[1 * q1 + 0];
            //
            // px_1^0 += py_0^1 * pf_01^0 + py_1^1 * pf_11^0
            //        += py_1^1 * x_0^1
            px[1 * q1 + 0] += py[1 * q1 + 1] * tx[0 * q1 + 1];
            //
            // px_1^1 += py_0^1 * pf_01^1 + py_1^1 * pf_11^1
            //        += py_1^1 * x_0^0
            px[1 * q1 + 1] += py[1 * q1 + 1] * tx[0 * q1 + 0];
            //
            // px_2^0 += py_0^1 * pf_02^0 + py_1^1 * pf_12^0
            //        += py_0^1 * 2 * x_2^1
            px[2 * q1 + 0] += py[0 * q1 + 1] * 2.0 * tx[2 * q1 + 1];
            //
            // px_2^1 += py_0^1 * pf_02^1 + py_1^1 * pf_12^1
            //        += py_0^1 * 2 * x_2^0
            px[2 * q1 + 1] += py[0 * q1 + 1] * 2.0 * tx[2 * q1 + 0];
        }
        // --------------------------------------------------------------
        // First order reverse computes partials of zero order coefficients
        // We use the notation pf_ij for partial of F_i^0 w.r.t. x_j^0
        //
        // y_0^0 = x_2^0 * x_2^0
        // pf_00 = 0,     pf_01 = 0,  pf_02 = 2 * x_2^0
        //
        // y_1^0 = x_0^0 * x_1^0
        // pf_10 = x_1^0, pf_11 = x_0^0,  pf_12 = 0
        //
        // px_0^0 += py_0^0 * pf_00 + py_1^0 * pf_10
        //        += py_1^0 * x_1^0
        px[0 * q1 + 0] += py[1 * q1 + 0] * tx[1 * q1 + 0];
        //
        // px_1^0 += py_1^0 * pf_01 + py_1^0 * pf_11
        //        += py_1^0 * x_0^0
        px[1 * q1 + 0] += py[1 * q1 + 0] * tx[0 * q1 + 0];
        //
        // px_2^0 += py_1^0 * pf_02 + py_1^0 * pf_12
        //        += py_0^0 * 2.0 * x_2^0
        px[2 * q1 + 0] += py[0 * q1 + 0] * 2.0 * tx[2 * q1 + 0];
        // --------------------------------------------------------------
        return ok;
    }
};
}  // End empty namespace
/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool reverse(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    // Create the atomic_reverse object
    atomic_reverse afun("atomic_reverse");
    //
    // Create the function f(u)
    //
    // domain space vector
    size_t n  = 3;
    double x_0 = 1.00;
    double x_1 = 2.00;
    double x_2 = 3.00;
    vector< AD<double> > au(n);
    au[0] = x_0;
    au[1] = x_1;
    au[2] = x_2;

    // declare independent variables and start tape recording
    CppAD::Independent(au);

    // range space vector
    size_t m = 2;
    vector< AD<double> > ay(m);

    // call atomic function
    vector< AD<double> > ax = au;
    afun(ax, ay);

    // create f: u -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (au, ay);  // y = f(u)
    //
    // check function value
    double check = x_2 * x_2;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);
    check = x_0 * x_1;
    ok &= NearEqual( Value(ay[1]) , check,  eps, eps);

    // --------------------------------------------------------------------
    // zero order forward
    //
    vector<double> x0(n), y0(m);
    x0[0] = x_0;
    x0[1] = x_1;
    x0[2] = x_2;
    y0   = f.Forward(0, x0);
    check = x_2 * x_2;
    ok &= NearEqual(y0[0] , check,  eps, eps);
    check = x_0 * x_1;
    ok &= NearEqual(y0[1] , check,  eps, eps);
    // --------------------------------------------------------------------
    // first order reverse
    //
    // value of Jacobian of f
    double check_jac[] = {
        0.0, 0.0, 2.0 * x_2,
        x_1, x_0,       0.0
    };
    vector<double> w(m), dw(n);
    //
    // check derivative of f_0 (x)
    for(size_t i = 0; i < m; i++)
    {   w[i]   = 1.0;
        w[1-i] = 0.0;
        dw = f.Reverse(1, w);
        for(size_t j = 0; j < n; j++)
        {   // compute partial in j-th component direction
            ok &= NearEqual(dw[j], check_jac[i * n + j], eps, eps);
        }
    }
    // --------------------------------------------------------------------
    // second order reverse
    //
    // value of Hessian of f_0
    double check_hes_0[] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 2.0
    };
    //
    // value of Hessian of f_1
    double check_hes_1[] = {
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };
    vector<double> x1(n), dw2( 2 * n );
    for(size_t j = 0; j < n; j++)
    {   for(size_t j1 = 0; j1 < n; j1++)
            x1[j1] = 0.0;
        x1[j] = 1.0;
        // first order forward
        f.Forward(1, x1);
        w[0] = 1.0;
        w[1] = 0.0;
        dw2  = f.Reverse(2, w);
        for(size_t i = 0; i < n; i++)
            ok &= NearEqual(dw2[i * 2 + 1], check_hes_0[i * n + j], eps, eps);
        w[0] = 0.0;
        w[1] = 1.0;
        dw2  = f.Reverse(2, w);
        for(size_t i = 0; i < n; i++)
            ok &= NearEqual(dw2[i * 2 + 1], check_hes_1[i * n + j], eps, eps);
    }
    // --------------------------------------------------------------------
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
