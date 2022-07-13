/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availabilitaylor_y set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_three_reverse.cpp$$
$spell
    Jacobian
$$

$section Atomic Functions and Reverse Mode: Example and Test$$

$head Purpose$$
This example demonstrates reverse mode derivative calculation
using an $cref atomic_three$$ function.

$head Function$$
For this example, the atomic function
$latex g : \B{R}^3 \rightarrow \B{R}^2$$ is defined by
$latex \[
g(x) = \left( \begin{array}{c}
    x_2 * x_2 \\
    x_0 * x_1
\end{array} \right)
\] $$

$head Jacobian$$
The corresponding Jacobian is
$latex \[
g^{(1)} (x) = \left( \begin{array}{ccc}
  0  &   0 & 2 x_2 \\
x_1  & x_0 & 0
\end{array} \right)
\] $$

$head Hessian$$
The Hessians of the component functions are
$latex \[
g_0^{(2)} ( x ) = \left( \begin{array}{ccc}
    0 & 0 & 0  \\
    0 & 0 & 0  \\
    0 & 0 & 2
\end{array} \right)
\W{,}
g_1^{(2)} ( x ) = \left( \begin{array}{ccc}
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
class atomic_reverse : public CppAD::atomic_three<double> {
/* %$$
$head Constructor $$
$srccode%cpp% */
public:
    atomic_reverse(const std::string& name) :
    CppAD::atomic_three<double>(name)
    { }
private:
/* %$$
$head for_type$$
$srccode%cpp% */
    // calculate type_y
    bool for_type(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        vector<CppAD::ad_type_enum>&        type_y      ) override
    {   assert( parameter_x.size() == type_x.size() );
        bool ok = type_x.size() == 3; // n
        ok     &= type_y.size() == 2; // m
        if( ! ok )
            return false;
        type_y[0] = type_x[2];
        type_y[1] = std::max(type_x[0], type_x[1]);
        return true;
    }
/* %$$
$head forward$$
$srccode%cpp% */
    // forward mode routine called by CppAD
    bool forward(
        const vector<double>&                   parameter_x ,
        const vector<CppAD::ad_type_enum>&      type_x      ,
        size_t                                  need_y      ,
        size_t                                  order_low   ,
        size_t                                  order_up    ,
        const vector<double>&                   taylor_x    ,
        vector<double>&                         taylor_y    ) override
    {
        size_t q1 = order_up + 1;
# ifndef NDEBUG
        size_t n = taylor_x.size() / q1;
        size_t m = taylor_y.size() / q1;
# endif
        assert( n == 3 );
        assert( m == 2 );
        assert( order_low <= order_up );

        // this example only implements up to first order forward mode
        bool ok = order_up <= 1;
        if( ! ok )
            return ok;

        // ------------------------------------------------------------------
        // Zero forward mode.
        // This case must always be implemented
        // g(x) = [ x_2 * x_2 ]
        //        [ x_0 * x_1 ]
        // y^0  = f( x^0 )
        if( order_low <= 0 )
        {   // y_0^0 = x_2^0 * x_2^0
            taylor_y[0*q1+0] = taylor_x[2*q1+0] * taylor_x[2*q1+0];
            // y_1^0 = x_0^0 * x_1^0
            taylor_y[1*q1+0] = taylor_x[0*q1+0] * taylor_x[1*q1+0];
        }
        if( order_up <= 0 )
            return ok;
        // ------------------------------------------------------------------
        // First order one forward mode.
        // This case is needed if first order forward mode is used.
        // g'(x) = [   0,   0, 2 * x_2 ]
        //         [ x_1, x_0,       0 ]
        // y^1 =  f'(x^0) * x^1
        if( order_low <= 1 )
        {   // y_0^1 = 2 * x_2^0 * x_2^1
            taylor_y[0*q1+1] = 2.0 * taylor_x[2*q1+0] * taylor_x[2*q1+1];

            // y_1^1 = x_1^0 * x_0^1 + x_0^0 * x_1^1
            taylor_y[1*q1+1]  = taylor_x[1*q1+0] * taylor_x[0*q1+1];
            taylor_y[1*q1+1] += taylor_x[0*q1+0] * taylor_x[1*q1+1];
        }
        return ok;
    }
/* %$$
$head reverse$$
$srccode%cpp% */
    // reverse mode routine called by CppAD
    bool reverse(
        const vector<double>&               parameter_x ,
        const vector<CppAD::ad_type_enum>&  type_x      ,
        size_t                              order_up    ,
        const vector<double>&               taylor_x    ,
        const vector<double>&               taylor_y    ,
        vector<double>&                     partial_x   ,
        const vector<double>&               partial_y   ) override
    {
        size_t q1 = order_up + 1;
        size_t n = taylor_x.size() / q1;
# ifndef NDEBUG
        size_t m = taylor_y.size() / q1;
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
                partial_x[j * q1 + k] = 0.0;
        //
        if( q1 == 2 )
        {   // --------------------------------------------------------------
            // Second order reverse first compute partials of first order
            // We use the notation pg_ij^k for partial of F_i^1 w.r.t. x_j^k
            //
            // y_0^1    = 2 * x_2^0 * x_2^1
            // pg_02^0  = 2 * x_2^1
            // pg_02^1  = 2 * x_2^0
            //
            // y_1^1    = x_1^0 * x_0^1 + x_0^0 * x_1^1
            // pg_10^0  = x_1^1
            // pg_11^0  = x_0^1
            // pg_10^1  = x_1^0
            // pg_11^1  = x_0^0
            //
            // px_0^0 += py_0^1 * pg_00^0 + py_1^1 * pg_10^0
            //        += py_1^1 * x_1^1
            partial_x[0*q1+0] += partial_y[1*q1+1] * taylor_x[1*q1+1];
            //
            // px_0^1 += py_0^1 * pg_00^1 + py_1^1 * pg_10^1
            //        += py_1^1 * x_1^0
            partial_x[0*q1+1] += partial_y[1*q1+1] * taylor_x[1*q1+0];
            //
            // px_1^0 += py_0^1 * pg_01^0 + py_1^1 * pg_11^0
            //        += py_1^1 * x_0^1
            partial_x[1*q1+0] += partial_y[1*q1+1] * taylor_x[0*q1+1];
            //
            // px_1^1 += py_0^1 * pg_01^1 + py_1^1 * pg_11^1
            //        += py_1^1 * x_0^0
            partial_x[1*q1+1] += partial_y[1*q1+1] * taylor_x[0*q1+0];
            //
            // px_2^0 += py_0^1 * pg_02^0 + py_1^1 * pg_12^0
            //        += py_0^1 * 2 * x_2^1
            partial_x[2*q1+0] += partial_y[0*q1+1] * 2.0 * taylor_x[2*q1+1];
            //
            // px_2^1 += py_0^1 * pg_02^1 + py_1^1 * pg_12^1
            //        += py_0^1 * 2 * x_2^0
            partial_x[2*q1+1] += partial_y[0*q1+1] * 2.0 * taylor_x[2*q1+0];
        }
        // --------------------------------------------------------------
        // First order reverse computes partials of zero order coefficients
        // We use the notation pg_ij for partial of F_i^0 w.r.t. x_j^0
        //
        // y_0^0 = x_2^0 * x_2^0
        // pg_00 = 0,     pg_01 = 0,  pg_02 = 2 * x_2^0
        //
        // y_1^0 = x_0^0 * x_1^0
        // pg_10 = x_1^0, pg_11 = x_0^0,  pg_12 = 0
        //
        // px_0^0 += py_0^0 * pg_00 + py_1^0 * pg_10
        //        += py_1^0 * x_1^0
        partial_x[0*q1+0] += partial_y[1*q1+0] * taylor_x[1*q1+0];
        //
        // px_1^0 += py_1^0 * pg_01 + py_1^0 * pg_11
        //        += py_1^0 * x_0^0
        partial_x[1*q1+0] += partial_y[1*q1+0] * taylor_x[0*q1+0];
        //
        // px_2^0 += py_1^0 * pg_02 + py_1^0 * pg_12
        //        += py_0^0 * 2.0 * x_2^0
        partial_x[2*q1+0] += partial_y[0*q1+0] * 2.0 * taylor_x[2*q1+0];
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
    // Create the atomic_reverse object corresponding to g(x)
    atomic_reverse afun("atomic_reverse");
    //
    // Create the function f(u) = g(u) for this example.
    //
    // domain space vector
    size_t n  = 3;
    double u_0 = 1.00;
    double u_1 = 2.00;
    double u_2 = 3.00;
    vector< AD<double> > au(n);
    au[0] = u_0;
    au[1] = u_1;
    au[2] = u_2;

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
    double check = u_2 * u_2;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);
    check = u_0 * u_1;
    ok &= NearEqual( Value(ay[1]) , check,  eps, eps);

    // --------------------------------------------------------------------
    // zero order forward
    //
    vector<double> u0(n), y0(m);
    u0[0] = u_0;
    u0[1] = u_1;
    u0[2] = u_2;
    y0   = f.Forward(0, u0);
    check = u_2 * u_2;
    ok &= NearEqual(y0[0] , check,  eps, eps);
    check = u_0 * u_1;
    ok &= NearEqual(y0[1] , check,  eps, eps);
    // --------------------------------------------------------------------
    // first order reverse
    //
    // value of Jacobian of f
    double check_jac[] = {
        0.0, 0.0, 2.0 * u_2,
        u_1, u_0,       0.0
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
    vector<double> u1(n), dw2( 2 * n );
    for(size_t j = 0; j < n; j++)
    {   for(size_t j1 = 0; j1 < n; j1++)
            u1[j1] = 0.0;
        u1[j] = 1.0;
        // first order forward
        f.Forward(1, u1);
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
