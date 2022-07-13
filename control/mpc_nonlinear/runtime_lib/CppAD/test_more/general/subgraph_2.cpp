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

namespace { // BEGIN_EMPTY_NAMESPACE

bool test_subgraph_subset(void)
{   bool ok = true;
    using std::cout;
    using CppAD::AD;
    using CppAD::vector;
    typedef vector<double> d_vector;
    typedef vector<size_t> s_vector;
    //
    size_t n = 4;
    d_vector x(n);
    vector< AD<double> > ax(n), ay(n);
    for(size_t j = 0; j < n; ++j)
        ax[j] = x[j] = double(j);
    CppAD::Independent(ax);
    for(size_t i = 0; i < n; ++i)
    {   ay[i] = 0.0;
        for(size_t j = 0; j < n; ++j)
            ay[i] += double(i + j + 1) * ax[j];
    }
    CppAD::ADFun<double> f(ax, ay);
    //
    size_t nnz = (n * (n + 1)) / 2;
    CppAD::sparse_rc<s_vector> upper_triangle(n, n, nnz);
    size_t k = 0;
    for(size_t i = 0; i < n; ++i)
    {   for(size_t j = i; j < n; ++j)
            upper_triangle.set(k++, i, j);
    }
    ok &= k == nnz;
    CppAD::sparse_rcv<s_vector, d_vector> subset( upper_triangle );
    //
    f.subgraph_jac_rev(x, subset);
    const d_vector& val = subset.val();
    k = 0;
    for(size_t i = 0; i < n; ++i)
    {   for(size_t j = i; j < n; ++j)
            ok &= val[k++] == double(i + j + 1);
    }
    ok &= k == nnz;
    //
    return ok;
}

} // END_EMPTY_NAMESPACE

bool subgraph_2(void)
{   bool ok = true;
    ok &= test_subgraph_subset();
    return ok;
}
