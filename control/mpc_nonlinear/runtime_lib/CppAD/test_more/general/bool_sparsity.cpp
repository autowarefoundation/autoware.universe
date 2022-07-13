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
Old example / test
@begin bool_sparsity.cpp$$
$spell
    Bool
$$

$section Using vectorBool Sparsity To Conserve Memory: Example and Test$$

$head Purpose$$
This example show how to conserve memory when computing sparsity patterns.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

@end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace {
    using CppAD::vector;
    using std::cout;
    using CppAD::vectorBool;
    using CppAD::AD;
    using CppAD::ADFun;

    // function f(x) that we are computing sparsity patterns for
    template <class Float>
    vector<Float> fun(const vector<Float>& x)
    {   size_t n  = x.size();
        vector<Float> ret(n + 1);
        for(size_t j = 0; j < n; j++)
        {   size_t k = (j + 1) % n;
            ret[j] = x[j] * x[j] * x[k];
        }
        ret[n] = 0.0;
        return ret;
    }
    // check sparsity pattern for f(x)
    bool check_jac(const vectorBool& pattern, size_t n)
    {   bool ok = true;
        for(size_t i = 0; i < n; i++)
        {   size_t k = (i + 1) % n;
            for(size_t j = 0; j < n; j++)
            {   bool non_zero = (i == j) || (j == k);
                ok &= pattern[ i * n + j] == non_zero;
            }
        }
        for(size_t j = 0; j < n; j++)
            ok &= pattern[ n * n + j] == false;
        return ok;
    }
    // check sparsity pattern for the Hessian of sum_i f_i(x)
    bool check_hes(const vectorBool& pattern, size_t n)
    {   bool ok = true;
        for(size_t i = 0; i < n; i++)
        {   size_t k1 = (i + 1) % n;
            size_t k2 = (n + i - 1) % n;
            for(size_t j = 0; j < n; j++)
            {   bool non_zero = (i == j) || (j == k1) || (j == k2);
                ok &= pattern[ i * n + j] == non_zero;
            }
        }
        return ok;
    }
    // compute sparsity for Jacobian of f(x) using forward mode
    bool for_sparse_jac(ADFun<double>& f)
    {   bool ok = true;
        size_t n = f.Domain();
        size_t m = f.Range();
        //
        // number of columns of the sparsity patter to compute at a time
        size_t n_col = vectorBool::bit_per_unit();
        vectorBool pattern(m * n), s(m * n_col), r(n * n_col);
        //
        size_t n_loop = (n - 1) / n_col + 1;
        for(size_t i_loop = 0; i_loop < n_loop; i_loop++)
        {   size_t j_col = i_loop * n_col;

            for(size_t i = 0; i < n; i++)
            {   for(size_t j = 0; j < n_col; j++)
                    r[i * n_col + j] = (i == j_col + j);
            }
            s = f.ForSparseJac(n_col, r);
            for(size_t i = 0; i < m; i++)
            {   for(size_t j = 0; j < n_col; j++)
                    if( j_col + j < n )
                        pattern[ i * n + j_col + j ] = s[ i * n_col + j];
            }
        }
        ok &= check_jac(pattern, n);
        //
        return ok;
    }
    // compute sparsity for Jacobian of f(x) using reverse mode
    bool rev_sparse_jac(ADFun<double>& f)
    {   bool ok = true;
        size_t n = f.Domain();
        size_t m = f.Range();
        //
        // number of rows of the sparsity patter to compute at a time
        size_t n_row = vectorBool::bit_per_unit();
        vectorBool pattern(m * n), s(n_row * n), r(n_row * m);
        //
        size_t n_loop = (m - 1) / n_row + 1;
        for(size_t i_loop = 0; i_loop < n_loop; i_loop++)
        {   size_t i_row = i_loop * n_row;

            for(size_t i = 0; i < n_row; i++)
            {   for(size_t j = 0; j < m; j++)
                    r[i * m + j] = (i_row + i == j);
            }
            s = f.RevSparseJac(n_row, r);
            for(size_t i = 0; i < n_row; i++)
            {   for(size_t j = 0; j < n; j++)
                    if( i_row + i < m )
                        pattern[ (i_row + i) * n + j ] = s[ i * n + j];
            }
        }
        ok &= check_jac(pattern, n);
        //
        return ok;
    }
    // compute sparsity for Hessian of sum_i f_i (x)
    bool rev_sparse_hes(ADFun<double>& f)
    {   bool ok = true;
        size_t n = f.Domain();
        size_t m = f.Range();
        //
        // number of columns of the sparsity patter to compute at a time
        size_t n_col = vectorBool::bit_per_unit();
        vectorBool pattern(n * n), r(n * n_col), h(n * n_col);

        // consider case where Hessian for sum of f_i(x) w.r.t i
        vectorBool s(m);
        for(size_t i = 0; i < m; i++)
            s[i] = true;
        //
        size_t n_loop = (n - 1) / n_col + 1;
        for(size_t i_loop = 0; i_loop < n_loop; i_loop++)
        {   size_t j_col = i_loop * n_col;

            for(size_t i = 0; i < n; i++)
            {   for(size_t j = 0; j < n_col; j++)
                    r[i * n_col + j] = (i == j_col + j);
            }
            //
            f.ForSparseJac(n_col, r);
            bool transpose = true;
            h = f.RevSparseHes(n_col, s, transpose);
            //
            for(size_t i = 0; i < n; i++)
            {   for(size_t j = 0; j < n_col; j++)
                    if( j_col + j < n )
                        pattern[ i * n + j_col + j ] = h[ i * n_col + j];
            }
        }
        ok &= check_hes(pattern, n);
        //
        return ok;
    }
}
// driver for all of the cases above
bool bool_sparsity(void)
{   bool ok = true;
    //
    // record the funcion
    size_t n = 100;
    size_t m = n + 1;
    vector< AD<double> > x(n), y(m);
    for(size_t j = 0; j < n; j++)
        x[j] = AD<double>(j+1);
    CppAD::Independent(x);
    y = fun(x);
    ADFun<double> f(x, y);
    //
    // run the three example / tests
    ok &= for_sparse_jac(f);
    ok &= rev_sparse_jac(f);
    ok &= rev_sparse_hes(f);
    return ok;
}
// END C++
