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
$begin code_gen_fun_sparse_jac_as_fun.cpp$$
$spell
    jacobian
$$

$section Pass Sparse Jacobian as Code Gen Function: Example and Test$$

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/example/code_gen_fun.hpp>

bool sparse_jac_as_fun(void)
{   bool ok = true;
    //
    typedef CppAD::cg::CG<double>     c_double;
    typedef CppAD::AD<c_double>      ac_double;
    //
    typedef CppAD::vector<size_t>     s_vector;
    typedef CppAD::vector<double>     d_vector;
    typedef CppAD::vector<ac_double> ac_vector;
    //
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    // domain space vector
    size_t n  = 2;
    ac_vector ac_x(n);
    for(size_t j = 0; j < n; ++j)
        ac_x[j] = 1.0 / double(j + 1);

    // declare independent variables and start tape recording
    CppAD::Independent(ac_x);

    // range space vector
    size_t m = 3;
    ac_vector ac_y(m);
    for(size_t i = 0; i < m; ++i)
        ac_y[i] = double(i + 1) * sin( ac_x[i % n] );

    // create f: x -> y and stop tape recording
    CppAD::ADFun<c_double> c_f(ac_x, ac_y);

    // create a version of f that evalutes using ac_double
    CppAD::ADFun<ac_double, c_double> ac_f = c_f.base2ad();

    // Independent varialbes while evaluating Jacobian
    CppAD::Independent(ac_x);

    // ----------------------------------------------------------------
    // Sparse Jacobian evaluation using any CppAD method
    // (there are lots of choices here)
    //
    // pattern_eye (pattern for identity matrix)
    CppAD::sparse_rc<s_vector> pattern_eye(n, n, n);
    for(size_t k = 0; k < n; ++k)
        pattern_eye.set(k, k, k);
    //
    // pattern_jac
    bool transpose     = false;
    bool dependency    = false;
    bool internal_bool = true;
    CppAD::sparse_rc<s_vector> pattern_jac;
    // note that c_f and ac_f have the same sparsity pattern
    c_f.for_jac_sparsity(
        pattern_eye, transpose, dependency, internal_bool, pattern_jac
    );
    //
    // ac_Jrcv
    CppAD::sparse_rcv<s_vector, ac_vector> ac_Jrcv( pattern_jac );
    CppAD::sparse_jac_work work;
    std::string coloring = "cppad";
    size_t group_max = n;
    ac_f.sparse_jac_for(
        group_max, ac_x, ac_Jrcv, pattern_jac, coloring, work
    );
    //
    // create g: x -> non-zero elements of Jacobian
    CppAD::ADFun<c_double> c_g(ac_x, ac_Jrcv.val());

    // create compiled version of c_g
    std::string file_name = "example_lib";
    code_gen_fun g(file_name, c_g);

    // evaluate the compiled jacobian
    d_vector x(n);
    for(size_t j = 0; j < n; ++j)
        x[j] = 1.0 / double(j + 2);
    d_vector val = g(x);

    // check Jaociban values
    size_t nnz = pattern_jac.nnz();
    const s_vector& row(pattern_jac.row());
    const s_vector& col(pattern_jac.col());
    s_vector row_major = pattern_jac.row_major();
    //
    size_t k = 0;
    ok &= val.size() == nnz;
    for(size_t i = 0; i < m; ++i)
    {   for(size_t j = 0; j < n; ++j)
        {   if( j == i % n )
            {   size_t ell = row_major[k];
                double check = double(i + 1) * cos( x[i % n] );
                ok &= i == row[ell];
                ok &= j == col[ell];
                ok &= CppAD::NearEqual(val[k], check, eps99, eps99);
                ++k;
            }
        }
    }
    ok &= k == nnz;
    return ok;
}
// END C++
