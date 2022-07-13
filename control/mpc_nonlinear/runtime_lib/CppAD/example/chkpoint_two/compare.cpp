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
$begin chkpoint_two_compare.cpp$$
$spell
    checkpointing
    Taylor
$$

$section Compare With and Without Checkpointing: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(AD<double>)            ADVector;
    typedef CPPAD_TESTVECTOR(size_t)                size_vector;

    void f_algo(const ADVector& y, ADVector& z)
    {   z[0] = 0.0;
        z[1] = 0.0;
        for(size_t k = 0; k < 3; k++)
        {   z[0] += y[0];
            z[1] += y[1];
        }
        return;
    }
    void g_algo(const ADVector& x, ADVector& y)
    {   y[0] = 1.0;
        y[1] = 1.0;
        for(size_t k = 0; k < 3; k++)
        {   y[0] *= x[0];
            y[1] *= x[1];
        }
        return;
    }
    bool equal(
       const CppAD::sparse_rc<size_vector>& pattern_left  ,
       const CppAD::sparse_rc<size_vector>& pattern_right )
    {
        size_vector row_major_left = pattern_left.row_major();
        size_vector row_major_right = pattern_right.row_major();
        bool ok = pattern_left.nnz() == pattern_right.nnz();
        if( ! ok )
            return ok;
        for(size_t k = 0; k < pattern_left.nnz(); ++k)
        {   size_t r_left = pattern_left.row()[ row_major_left[k] ];
            size_t c_left = pattern_left.col()[ row_major_left[k] ];
            size_t r_right = pattern_right.row()[ row_major_right[k] ];
            size_t c_right = pattern_right.col()[ row_major_right[k] ];
            ok &= (r_left == r_right) && (c_left == c_right);
        }
        return ok;
    }
}
bool compare(void)
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
    bool use_base2ad      = false;
    bool use_in_parallel  = false;
    CppAD::chkpoint_two<double> f_chk(f_fun, "f_chk",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );
    CppAD::chkpoint_two<double> g_chk(g_fun, "g_chk",
        internal_bool, use_hes_sparsity, use_base2ad, use_in_parallel
    );

    // Record a version of z = f[g(x)] without checkpointing
    Independent(ax);
    g_algo(ax, ay);
    f_algo(ay, az);
    CppAD::ADFun<double> check_not(ax, az);

    // Record a version of z = f[g(x)] with checkpointing
    Independent(ax);
    g_chk(ax, ay);
    f_chk(ay, az);
    CppAD::ADFun<double> check_yes(ax, az);

    // checkpointing should use fewer operations
    ok &= check_not.size_var() > check_yes.size_var();

    // this does not really save space because f and g are only used once
    ok &= check_not.size_var() <= check_yes.size_var()
        + f_fun.size_var() + g_fun.size_var();

    // compare forward mode results for orders 0, 1, 2
    size_t q1 = 3; // order_up + 1
    CPPAD_TESTVECTOR(double) x_q(nx*q1), z_not(nz*q1), z_yes(nz*q1);
    for(size_t j = 0; j < nx; j++)
    {   for(size_t k = 0; k < q1; k++)
            x_q[ j * q1 + k ] = 1.0 / double(q1 - k);
    }
    z_not = check_not.Forward(q1-1, x_q);
    z_yes = check_yes.Forward(q1-1, x_q);
    for(size_t i = 0; i < nz; i++)
    {   for(size_t k = 0; k < q1; k++)
        {   double zik_not = z_not[ i * q1 + k];
            double zik_yes = z_yes[ i * q1 + k];
            ok &= NearEqual(zik_not, zik_yes, eps99, eps99);
        }
    }

    // compare reverse mode results for orders 0, 1, 2
    CPPAD_TESTVECTOR(double) w(nz*q1), dw_not(nx*q1), dw_yes(nx*q1);
    for(size_t i = 0; i < nz * q1; i++)
        w[i] = 1.0 / double(i + 1);
    dw_not = check_not.Reverse(q1, w);
    dw_yes = check_yes.Reverse(q1, w);
    for(size_t j = 0; j < nx; j++)
    {   for(size_t k = 0; k < q1; k++)
        {   double dwjk_not = dw_not[ j * q1 + k];
            double dwjk_yes = dw_yes[ j * q1 + k];
            ok &= NearEqual(dwjk_not, dwjk_yes, eps99, eps99);
        }
    }

    // compare Jacobian sparsity patterns
    CppAD::sparse_rc<size_vector> pattern_in, pattern_not, pattern_yes;
    pattern_in.resize(nx, nx, nx);
    for(size_t k = 0; k < nx; ++k)
        pattern_in.set(k, k, k);
    bool transpose     = false;
    bool dependency    = false;
    internal_bool      = false;
    // for_jac_sparsity (not internal_bool is false)
    check_not.for_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_not
    );
    pattern_in.resize(nz, nz, nz);
    for(size_t k = 0; k < nz; ++k)
        pattern_in.set(k, k, k);
    // forward and reverse Jacobian sparsity should give same answer
    check_yes.rev_jac_sparsity(
        pattern_in, transpose, dependency, internal_bool, pattern_yes
    );
    ok &= equal(pattern_not, pattern_yes );

    // compare Hessian sparsity patterns
    CPPAD_TESTVECTOR(bool) select_x(nx), select_z(nz);
    for(size_t j = 0; j < nx; ++j)
        select_x[j] = true;
    for(size_t i = 0; i < nz; ++i)
        select_z[i] = true;
    transpose       = false;
    // Reverse should give same results as forward because
    // previous for_jac_sparsity used identity for pattern_in.
    // Note that internal_bool must be same as in call to for_sparse_jac.
    check_not.rev_hes_sparsity(
        select_z, transpose, internal_bool, pattern_yes
    );
    // internal_bool need not be the same during a call to for_hes_sparsity
    internal_bool = ! internal_bool;
    check_yes.for_hes_sparsity(
        select_x, select_z, internal_bool, pattern_not
    );
    ok &= equal(pattern_not, pattern_yes);
    //
    return ok;
}
// END C++
