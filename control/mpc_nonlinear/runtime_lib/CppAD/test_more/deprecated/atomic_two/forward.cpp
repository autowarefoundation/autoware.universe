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
$begin atomic_two_forward.cpp$$
$spell
    Jacobian
$$

$section Atomic Forward: Example and Test$$

$head Purpose$$
This example demonstrates forward mode derivative calculation
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
class atomic_forward : public CppAD::atomic_base<double> {
/* %$$
$head Constructor $$
$srccode%cpp% */
public:
    // constructor (could use const char* for name)
    atomic_forward(const std::string& name) :
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

        // this example only implements up to second order forward mode
        bool ok = q <= 2;
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
        if( q <= 1 )
            return ok;
        // ------------------------------------------------------------------
        // Second order forward mode.
        // This case is neede if second order forwrd mode is used.
        // f'(x) = [   0,   0, 2 x_2 ]
        //         [ x_1, x_0,     0 ]
        //
        //            [ 0 , 0 , 0 ]                  [ 0 , 1 , 0 ]
        // f_0''(x) = [ 0 , 0 , 0 ]  f_1^{(2)} (x) = [ 1 , 0 , 0 ]
        //            [ 0 , 0 , 2 ]                  [ 0 , 0 , 0 ]
        //
        //  y_0^2 = x^1 * f_0''( x^0 ) x^1 / 2! + f_0'( x^0 ) x^2
        //        = ( x_2^1 * 2.0 * x_2^1 ) / 2!
        //        + 2.0 * x_2^0 * x_2^2
        ty[0 * q1 + 2]  = tx[2 * q1 + 1] * tx[2 * q1 + 1];
        ty[0 * q1 + 2] += 2.0 * tx[2 * q1 + 0] * tx[2 * q1 + 2];
        //
        //  y_1^2 = x^1 * f_1''( x^0 ) x^1 / 2! + f_1'( x^0 ) x^2
        //        = ( x_1^1 * x_0^1 + x_0^1 * x_1^1) / 2
        //        + x_1^0 * x_0^2 + x_0^0 + x_1^2
        ty[1 * q1 + 2]  = tx[1 * q1 + 1] * tx[0 * q1 + 1];
        ty[1 * q1 + 2] += tx[1 * q1 + 0] * tx[0 * q1 + 2];
        ty[1 * q1 + 2] += tx[0 * q1 + 0] * tx[1 * q1 + 2];
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
    // Create the atomic_forward object
    atomic_forward afun("atomic_forward");
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
    // first order forward
    //
    // value of Jacobian of f
    double check_jac[] = {
        0.0, 0.0, 2.0 * x_2,
        x_1, x_0,       0.0
    };
    vector<double> x1(n), y1(m);
    // check first order forward mode
    for(size_t j = 0; j < n; j++)
        x1[j] = 0.0;
    for(size_t j = 0; j < n; j++)
    {   // compute partial in j-th component direction
        x1[j] = 1.0;
        y1    = f.Forward(1, x1);
        x1[j] = 0.0;
        // check this direction
        for(size_t i = 0; i < m; i++)
            ok &= NearEqual(y1[i], check_jac[i * n + j], eps, eps);
    }
    // --------------------------------------------------------------------
    // second order forward
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
    vector<double> x2(n), y2(m);
    for(size_t j = 0; j < n; j++)
        x2[j] = 0.0;
    // compute diagonal elements of the Hessian
    for(size_t j = 0; j < n; j++)
    {   // first order forward in j-th direction
        x1[j] = 1.0;
        f.Forward(1, x1);
        y2 = f.Forward(2, x2);
        // check this element of Hessian diagonal
        ok &= NearEqual(y2[0], check_hes_0[j * n + j] / 2.0, eps, eps);
        ok &= NearEqual(y2[1], check_hes_1[j * n + j] / 2.0, eps, eps);
        //
        for(size_t k = 0; k < n; k++) if( k != j )
        {   x1[k] = 1.0;
            f.Forward(1, x1);
            y2 = f.Forward(2, x2);
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
            x1[k] = 0.0;
        }
        x1[j] = 0.0;
    }
    // --------------------------------------------------------------------
    return ok;
}
/* %$$
$$ $comment end nospell$$
$end
*/
