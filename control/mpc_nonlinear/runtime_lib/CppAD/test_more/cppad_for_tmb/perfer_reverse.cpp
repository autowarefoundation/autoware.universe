/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/cppad.hpp>
# include <omp.h>

namespace { // BEGIN_EMPTY_NAMESPACE

using CppAD::vector;

// ----------------------------------------------------------------------------
// prefer reverse mode during computation of Jacobians

// example_tmb_atomic
class example_tmb_atomic : public CppAD::atomic_base<double> {
public:
    // constructor
    example_tmb_atomic(const std::string& name)
    : CppAD::atomic_base<double>(name)
    { }
    // forward (only implement zero order)
    virtual bool forward(
        size_t                p  ,
        size_t                q  ,
        const vector<bool>&   vx ,
        vector<bool>&         vy ,
        const vector<double>& tx ,
        vector<double>&       ty )
    {
        // check for errors in usage
        bool ok = p == 0 && q == 0;
        ok     &= tx.size() == 1;
        ok     &= ty.size() == 1;
        ok     &= vx.size() <= 1;
        if( ! ok )
            return false;

        // variable information
        if( vx.size() > 0 )
            vy[0] = vx[0];

        // y = 1 / x
        ty[0] = 1.0 / tx[0];

        return ok;
    }
    // reverse (implement first order)
    virtual bool reverse(
        size_t                q  ,
        const vector<double>& tx ,
        const vector<double>& ty ,
        vector<double>&       px ,
        const vector<double>& py )
    {
        // check for errors in usage
        bool ok = q == 0;
        ok     &= tx.size() == 1;
        ok     &= ty.size() == 1;
        ok     &= px.size() == 1;
        ok     &= py.size() == 1;
        if( ! ok )
            return false;

        // y = 1 / x
        // dy/dx = - 1 / (x * x)
        double dy_dx = -1.0 / ( tx[0] * tx[0] );
        px[0]        = py[0] * dy_dx;

        return ok;
    }
};

} // END_EMPTY_NAMESPACE

bool prefer_reverse(void)
{   bool ok = true;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // Create atomic functions
    example_tmb_atomic afun("reciprocal");

    // Declare independent variables
    size_t n = 1;
    CPPAD_TESTVECTOR( CppAD::AD<double> ) ax(n);
    ax[0] = 5.0;
    CppAD::Independent(ax);

    // Compute dependent variables
    size_t m = 1;
    CPPAD_TESTVECTOR( CppAD::AD<double> ) ay(m);
    afun(ax, ay);

    // Create f(x) = 1 / x
    CppAD::ADFun<double> f(ax, ay);

    // Use Jacobian to compute f'(x) = - 1 / (x * x).
    // This would fail with the normal CppAD distribution because it would use
    // first order forward mode for the  calculation.
    CPPAD_TESTVECTOR(double) x(n), dy_dx(m);
    x[0]   = 2.0;
    dy_dx  = f.Jacobian(x);

    // check the result
    double check = -1.0 / (x[0] * x[0]);
    ok &= CppAD::NearEqual(dy_dx[0], check, eps99, eps99);

    return ok;
}
