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
$begin chkpoint_two_base2ad.cpp$$
$spell
    checkpointing
    Taylor
$$

$section Checkpointing With base2ad: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(AD<double>)            ADVector;
    typedef CPPAD_TESTVECTOR(size_t)                size_vector;

    // f(y) = ( 3*y[0], 3*y[1] )
    void f_algo(const ADVector& y, ADVector& z)
    {   z[0] = 0.0;
        z[1] = 0.0;
        for(size_t k = 0; k < 3; k++)
        {   z[0] += y[0];
            z[1] += y[1];
        }
        return;
    }
    // g(x) = ( x[0]^3, x[1]^3 )
    void g_algo(const ADVector& x, ADVector& y)
    {   y[0] = 1.0;
        y[1] = 1.0;
        for(size_t k = 0; k < 3; k++)
        {   y[0] *= x[0];
            y[1] *= x[1];
        }
        return;
    }
}
bool base2ad(void)
{   bool ok = true;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // AD vectors holding x, y, and z values
    size_t nx = 2, ny = 2, nz = 2;
    ADVector ax(nx), ay(ny), az(nz);

    // record the function g_fun(x)
    for(size_t j = 0; j < nx; j++)
        ax[j] = double(j + 1);
    Independent(ax);
    g_algo(ax, ay);
    CppAD::ADFun<double> g_fun(ax, ay);

    // record the function f_fun(y)
    Independent(ay);
    f_algo(ay, az);
    CppAD::ADFun<double> f_fun(ay, az);

    // create checkpoint versions of f and g
    bool internal_bool    = true;
    bool use_hes_sparsity = true;
    bool use_base2ad      = true;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> f_chk(f_fun, "f_chk",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    CppAD::chkpoint_two<double> g_chk(g_fun, "g_chk",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );

    // Record a version of z = f[g(x)] = h(x) with checkpointing
    // h(x) = [ 3*x[0]^3 , 3*x[1]^3 ]
    Independent(ax);
    g_chk(ax, ay);
    f_chk(ay, az);
    CppAD::ADFun<double> h_fun(ax, az);

    // Use base2ad to create and AD<double> verison of h
    CppAD::ADFun< AD<double>, double> ah_fun = h_fun.base2ad();

    // start recording AD<Base> operations
    Independent(ax);

    // record evaluate derivative of h_0 (x)
    az = ah_fun.Forward(0, ax);
    ADVector aw(nz), adw(nx);
    aw[0] = 1.0;
    for(size_t i = 1; i < nz; ++i)
        aw[i] = 0.0;
    adw = ah_fun.Reverse(1, aw);
    // k(x) = h_0 '(x) = [ 9*x[0]^2 , 0.0 ]
    CppAD::ADFun<double> k_fun(ax, adw);

    // Evaluate the Jacobian of k(x)
    CPPAD_TESTVECTOR(double) x(nx);
    for(size_t j = 0; j < nx; ++j)
        x[j] = 2.0 + double(nx - j);
    CPPAD_TESTVECTOR(double) J = k_fun.Jacobian(x);

    // check result
    for(size_t i = 0; i < nz; ++i)
    {   for(size_t j = 0; j < nx; ++j)
        {   double Jij = J[i * nx + j];
            if( i == 0 && j == 0 )
            {   double check = 18.0 * x[0];
                ok &= CppAD::NearEqual(Jij, check, eps99, eps99);
            }
            else
                ok &= Jij == 0.0;
        }
    }

    return ok;
}
// END C++
