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
$begin atomic_two_base2ad.cpp$$

$section base2ad with Atomic Operations: Example and Test$$

$head Source Code$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace {          // isolate items below to this file
//
// abbreviations
using CppAD::AD;
using CppAD::vector;
//
class atomic_base2ad : public CppAD::atomic_base<double> {
//
public:
    // constructor (could use const char* for name)
    atomic_base2ad(const std::string& name) :
    // this example does not use any sparsity patterns
    CppAD::atomic_base<double>(name)
    { }
private:
    // ----------------------------------------------------------------------
    // forward mode
    // ----------------------------------------------------------------------
    template <class Scalar>
    bool template_forward(
        size_t                    p ,
        size_t                    q ,
        const vector<bool>&      vx ,
              vector<bool>&      vy ,
        const vector<Scalar>&    tx ,
              vector<Scalar>&    ty
    )
    {
# ifndef NDEBUG
        size_t n = tx.size() / (q + 1);
        size_t m = ty.size() / (q + 1);
# endif
        assert( n == 1 );
        assert( m == 1 );

        // return flag
        bool ok = q == 0;
        if( ! ok )
            return ok;

        // check for defining variable information
        // This case must always be implemented
        if( vx.size() > 0 )
            vy[0] = vx[0];

        // Order zero forward mode.
        // This case must always be implemented
        // y^0 = f( x^0 ) = 1 / x^0
        Scalar f = 1. / tx[0];
        if( p <= 0 )
            ty[0] = f;
        return ok;
    }
    // forward mode routines called by ADFun<Base> objects
    virtual bool forward(
        size_t                    p ,
        size_t                    q ,
        const vector<bool>&      vx ,
              vector<bool>&      vy ,
        const vector<double>&    tx ,
              vector<double>&    ty
    )
    {   return template_forward(p, q, vx, vy, tx, ty);
    }
    // forward mode routines called by ADFun< AD<Base> , Base> objects
    virtual bool forward(
        size_t                          p ,
        size_t                          q ,
        const vector<bool>&            vx ,
              vector<bool>&            vy ,
        const vector< AD<double> >&    atx ,
              vector< AD<double> >&    aty
    )
    {   return template_forward(p, q, vx, vy, atx, aty);
    }
    // ----------------------------------------------------------------------
    // reverse mode
    // ----------------------------------------------------------------------
    template <class Scalar>
    bool template_reverse(
        size_t                    q ,
        const vector<Scalar>&    tx ,
        const vector<Scalar>&    ty ,
              vector<Scalar>&    px ,
        const vector<Scalar>&    py
    )
    {
# ifndef NDEBUG
        size_t n = tx.size() / (q + 1);
        size_t m = ty.size() / (q + 1);
# endif
        assert( n == 1 );
        assert( m == 1 );

        // return flag
        bool ok = q == 0;
        if( ! ok )
            return ok;

        // Order zero reverse mode.
        // y^0 = f( x^0 ) = 1 / x^0
        // y^1 = f'( x^0 ) * x^1 = - x^1 / (x^0 * x^0)
        px[0] = - py[0] / ( tx[0] * tx[0] );
        return ok;
    }
    // reverse mode routines called by ADFun<Base> objects
    virtual bool reverse(
        size_t                    q ,
        const vector<double>&    tx ,
        const vector<double>&    ty ,
              vector<double>&    px ,
        const vector<double>&    py
    )
    {   return template_reverse(q, tx, ty, px, py);
    }
    // reverse mode routines called by ADFun<Base> objects
    virtual bool reverse(
        size_t                         q ,
        const vector< AD<double> >&    atx ,
        const vector< AD<double> >&    aty ,
              vector< AD<double> >&    apx ,
        const vector< AD<double> >&    apy
    )
    {   return template_reverse(q, atx, aty, apx, apy);
    }
}; // End of atomic_base2ad class
}  // End empty namespace

bool base2ad(void)
{   bool ok = true;
    using CppAD::NearEqual;
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();
    //
    // Create the atomic base2ad object
    atomic_base2ad afun("atomic_base2ad");
    //
    // Create the function f(x)
    //
    size_t n  = 1;
    double  x0 = 0.5;
    vector< AD<double> > ax(n);
    ax[0]     = x0;

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // range space vector
    size_t m = 1;
    vector< AD<double> > ay(m);

    // call atomic function and store base2ad(x) in au[0]
    vector< AD<double> > au(m);
    afun(ax, au);        // u = 1 / x

    // now use AD division to invert to invert the operation
    ay[0] = 1.0 / au[0]; // y = 1 / u = x

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f;
    f.Dependent (ax, ay);  // f(x) = x

    // check function value
    double check = x0;
    ok &= NearEqual( Value(ay[0]) , check,  eps, eps);

    // check zero order forward mode
    size_t q;
    vector<double> x_q(n), y_q(m);
    q      = 0;
    x_q[0] = x0;
    y_q    = f.Forward(q, x_q);
    ok &= NearEqual(y_q[0] , check,  eps, eps);

    // check first order reverse
    vector<double> dw(n), w(m);
    w[0]  = 1.0;
    dw    = f.Reverse(q+1, w);
    check = 1.0;
    ok &= NearEqual(dw[0] , check,  eps, eps);

    // create af : x -> y
    CppAD::ADFun< AD<double> , double > af( f.base2ad() );

    // check zero order forward mode
    vector< AD<double> > ax_q(n), ay_q(m);
    q      = 0;
    ax_q[0] = x0;
    ay_q    = af.Forward(q, ax_q);
    check   = x0;
    ok &= NearEqual( Value(ay_q[0]) , check,  eps, eps);

    // check first order reverse
    vector< AD<double> > adw(n), aw(m);
    aw[0]  = 1.0;
    adw    = af.Reverse(q+1, aw);
    check = 1.0;
    ok &= NearEqual( Value(adw[0]) , check,  eps, eps);

    return ok;
}
// END C++
