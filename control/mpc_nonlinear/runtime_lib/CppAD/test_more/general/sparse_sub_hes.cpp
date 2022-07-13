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
$begin sparse_sub_hes.cpp$$
$spell
$$

$section Sparse Hessian on Subset of Variables: Example and Test$$

$head Purpose$$
This example uses a
$cref/column subset/sparse_hessian/p/Column Subset/$$ of the sparsity pattern
to compute the Hessian for a subset of the variables.
The values in the rest of the sparsity pattern do not matter.

$head See Also$$
$cref sub_sparse_hes.cpp$$

$code
$comment%example/sparse/sparse_sub_hes.cpp%0%// BEGIN C++%// END C++%1%$$
$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace { // BEGIN_EMPTY_NAMESPACE

// --------------------------------------------------------------------------
void record_function(CppAD::ADFun<double>& f, size_t n)
{   // must be greater than or equal 3; see n_sweep below
    assert( n >= 3 );
    //
    using CppAD::AD;
    typedef CppAD::vector< AD<double> >     a_vector;
    //
    // domain space vector
    a_vector a_x(n);
    for(size_t j = 0; j < n; j++)
        a_x[j] = AD<double> (0);

    // declare independent variables and starting recording
    CppAD::Independent(a_x);

    // range space vector
    size_t m = 1;
    a_vector a_y(m);
    a_y[0] = 0.0;
    for(size_t j = 1; j < n; j++)
        a_y[0] += a_x[j-1] * a_x[j] * a_x[j];

    // create f: x -> y and stop tape recording
    // (without executing zero order forward calculation)
    f.Dependent(a_x, a_y);
    //
    return;
}
// --------------------------------------------------------------------------
bool test_set(const char* color_method)
{   bool ok = true;
    //
    typedef CppAD::vector< double >                   d_vector;
    typedef CppAD::vector<size_t>                     i_vector;
    typedef CppAD::vector< std::set<size_t> >         s_vector;
    //
    size_t n = 12;
    CppAD::ADFun<double> f;
    record_function(f, n);
    //
    // sparsity patteren for the sub-set of variables we are computing
    // the hessian w.r.t.
    size_t n_sub = 4;
    s_vector r(n);
    for(size_t j = 0; j < n_sub; j++)
    {   assert(  r[j].empty() );
        r[j].insert(j);
    }

    // store forward sparsity for J(x) = F^{(1)} (x) * R
    f.ForSparseJac(n_sub, r);

    // compute sparsity pattern for H(x) = (S * F)^{(2)} ( x ) * R
    s_vector s(1);
    assert(  s[0].empty() );
    s[0].insert(0);
    bool transpose = true;
    s_vector h = f.RevSparseHes(n_sub, s, transpose);

    // set the row and column indices that correspond to lower triangle
    i_vector row, col;
    for(size_t i = 0; i < n_sub; i++)
    {   if( i > 0 )
        {   // diagonal element
            row.push_back(i);
            col.push_back(i);
            // lower diagonal element
            row.push_back(i);
            col.push_back(i-1);
        }
    }

    // weighting for the Hessian
    d_vector w(1);
    w[0] = 1.0;

    // compute Hessian
    CppAD::sparse_hessian_work work;
    work.color_method = color_method;
    d_vector x(n), hes( row.size() );
    for(size_t j = 0; j < n; j++)
        x[j] = double(j+1);
    f.SparseHessian(x, w, h, row, col, hes, work);

    // check the values in the sparse hessian
    for(size_t ell = 0; ell < row.size(); ell++)
    {   size_t i = row[ell];
        size_t j = col[ell];
        if( i == j )
            ok &= hes[ell] == 2.0 * x[i-1];
        else
        {   ok &= j+1 == i;
            ok &= hes[ell] == 2.0 * x[i];
        }
    }
    return ok;
}
// --------------------------------------------------------------------------
bool test_bool(const char* color_method)
{   bool ok = true;
    //
    typedef CppAD::vector< double >    d_vector;
    typedef CppAD::vector<size_t>      i_vector;
    typedef CppAD::vector<bool>        s_vector;
    //
    size_t n = 12;
    CppAD::ADFun<double> f;
    record_function(f, n);
    //
    // sparsity patteren for the sub-set of variables we are computing
    // the hessian w.r.t.
    size_t n_sub = 4;
    s_vector r(n * n_sub);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n_sub; j++)
            r[ i * n_sub + j ] = (i == j);
    }

    // store forward sparsity for J(x) = F^{(1)} (x) * R
    f.ForSparseJac(n_sub, r);

    // compute sparsity pattern for H(x) = (S * F)^{(2)} ( x ) * R
    s_vector s(1);
    s[0] = true;
    bool transpose = true;
    s_vector h = f.RevSparseHes(n_sub, s, transpose);

    // set the row and column indices that correspond to lower triangle
    i_vector row, col;
    for(size_t i = 0; i < n_sub; i++)
    {   if( i > 0 )
        {   // diagonal element
            row.push_back(i);
            col.push_back(i);
            // lower diagonal element
            row.push_back(i);
            col.push_back(i-1);
        }
    }

    // weighting for the Hessian
    d_vector w(1);
    w[0] = 1.0;

    // extend sparsity pattern (values in extended columns do not matter)
    s_vector h_extended(n * n);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n_sub; j++)
            h_extended[ i * n + j ] = h[ i * n_sub + j ];
        for(size_t j = n_sub; j < n; j++)
            h_extended[ i * n + j ] = false;
    }
    // compute Hessian
    CppAD::sparse_hessian_work work;
    work.color_method = color_method;
    d_vector x(n), hes( row.size() );
    for(size_t j = 0; j < n; j++)
        x[j] = double(j+1);
    f.SparseHessian(x, w, h_extended, row, col, hes, work);

    // check the values in the sparse hessian
    for(size_t ell = 0; ell < row.size(); ell++)
    {   size_t i = row[ell];
        size_t j = col[ell];
        if( i == j )
            ok &= hes[ell] == 2.0 * x[i-1];
        else
        {   ok &= j+1 == i;
            ok &= hes[ell] == 2.0 * x[i];
        }
    }
    return ok;
}
} // END_EMPTY_NAMESPACE

bool sparse_sub_hes(void)
{   bool ok = true;
    ok &= test_set("cppad.symmetric");
    ok &= test_set("cppad.general");
    //
    ok &= test_bool("cppad.symmetric");
    ok &= test_bool("cppad.general");
    return ok;
}
// END C++
