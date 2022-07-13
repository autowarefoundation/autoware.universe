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
$begin atomic_three_forward.cpp$$
$spell
    Jacobian
$$

$section Atomic Functions and Forward Mode: Example and Test$$

$head Purpose$$
This example demonstrates forward mode derivative calculation
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
namespace {          // begin empty namespace
using CppAD::vector; // abbreviate CppAD::vector using vector
//
class atomic_forward : public CppAD::atomic_three<double> {
/* %$$
$head Constructor $$
$srccode%cpp% */
public:
    atomic_forward(const std::string& name) :
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
        const vector<double>&              parameter_x  ,
        const vector<CppAD::ad_type_enum>& type_x       ,
        size_t                             need_y       ,
        size_t                             order_low    ,
        size_t                             order_up     ,
        const vector<double>&              taylor_x     ,
        vector<double>&                    taylor_y     ) override
    {
        size_t q1 = order_up + 1;
# ifndef NDEBUG
        size_t n = taylor_x.size() / q1;
        size_t m = taylor_y.size() / q1;
# endif
        assert( n == 3 );
        assert( m == 2 );
        assert( order_low <= order_up );

        // this example only implements up to second order forward mode
        bool ok = order_up <=  2;
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
        if( order_up <=  0 )
            return ok;
        // ------------------------------------------------------------------
        // First order forward mode.
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
        if( order_up <=  1 )
            return ok;
        // ------------------------------------------------------------------
        // Second order forward mode.
        // This case is neede if second order forwrd mode is used.
        // g'(x) = [   0,   0, 2 x_2 ]
        //         [ x_1, x_0,     0 ]
        //
        //            [ 0 , 0 , 0 ]                  [ 0 , 1 , 0 ]
        // g_0''(x) = [ 0 , 0 , 0 ]  g_1^{(2)} (x) = [ 1 , 0 , 0 ]
        //            [ 0 , 0 , 2 ]                  [ 0 , 0 , 0 ]
        //
        //  y_0^2 = x^1 * g_0''( x^0 ) x^1 / 2! + g_0'( x^0 ) x^2
        //        = ( x_2^1 * 2.0 * x_2^1 ) / 2!
        //        + 2.0 * x_2^0 * x_2^2
        taylor_y[0*q1+2]  = taylor_x[2*q1+1] * taylor_x[2*q1+1];
        taylor_y[0*q1+2] += 2.0 * taylor_x[2*q1+0] * taylor_x[2*q1+2];
        //
        //  y_1^2 = x^1 * g_1''( x^0 ) x^1 / 2! + g_1'( x^0 ) x^2
        //        = ( x_1^1 * x_0^1 + x_0^1 * x_1^1) / 2
        //        + x_1^0 * x_0^2 + x_0^0 + x_1^2
        taylor_y[1*q1+2]  = taylor_x[1*q1+1] * taylor_x[0*q1+1];
        taylor_y[1*q1+2] += taylor_x[1*q1+0] * taylor_x[0*q1+2];
        taylor_y[1*q1+2] += taylor_x[0*q1+0] * taylor_x[1*q1+2];
        // ------------------------------------------------------------------
        return ok;
    }
};
}  // End empty namespace
/* %$$
$head Use Atomic Function$$
$srccode%cpp% */
bool forward(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    // Create the atomic_forward object corresponding to g(x)
    atomic_forward afun("atomic_forward");
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
    // first order forward
    //
    // value of Jacobian of f
    double check_jac[] = {
        0.0, 0.0, 2.0 * u_2,
        u_1, u_0,       0.0
    };
    vector<double> u1(n), y1(m);
    // check first order forward mode
    for(size_t j = 0; j < n; j++)
        u1[j] = 0.0;
    for(size_t j = 0; j < n; j++)
    {   // compute partial in j-th component direction
        u1[j] = 1.0;
        y1    = f.Forward(1, u1);
        u1[j] = 0.0;
        // check this direction
        for(size_t i = 0; i < m; i++)
            ok &= NearEqual(y1[i], check_jac[i * n + j], eps, eps);
    }
    // --------------------------------------------------------------------
    // second order forward
    //
    // value of Hessian of g_0
    double check_hes_0[] = {
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 2.0
    };
    //
    // value of Hessian of g_1
    double check_hes_1[] = {
        0.0, 1.0, 0.0,
        1.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };
    vector<double> u2(n), y2(m);
    for(size_t j = 0; j < n; j++)
        u2[j] = 0.0;
    // compute diagonal elements of the Hessian
    for(size_t j = 0; j < n; j++)
    {   // first order forward in j-th direction
        u1[j] = 1.0;
        f.Forward(1, u1);
        y2 = f.Forward(2, u2);
        // check this element of Hessian diagonal
        ok &= NearEqual(y2[0], check_hes_0[j * n + j] / 2.0, eps, eps);
        ok &= NearEqual(y2[1], check_hes_1[j * n + j] / 2.0, eps, eps);
        //
        for(size_t k = 0; k < n; k++) if( k != j )
        {   u1[k] = 1.0;
            f.Forward(1, u1);
            y2 = f.Forward(2, u2);
            //
            // y2 = (H_jj + H_kk + H_jk + H_kj) / 2.0
            // y2 = (H_jj + H_kk) / 2.0 + H_jk
            //
            double H_jj = check_hes_0[j * n + j];
            double H_kk = check_hes_0[k * n + k];
            double H_jk = y2[0] - (H_kk + H_jj) / 2.0;
            ok &= NearEqual(H_jk, check_hes_0[j * n + k], eps, eps);
            //
            H_jj = check_hes_1[j * n + j];
            H_kk = check_hes_1[k * n + k];
            H_jk = y2[1] - (H_kk + H_jj) / 2.0;
            ok &= NearEqual(H_jk, check_hes_1[j * n + k], eps, eps);
            //
            u1[k] = 0.0;
        }
        u1[j] = 0.0;
    }
    // --------------------------------------------------------------------
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
