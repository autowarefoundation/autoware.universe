/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

namespace { // ---------------------------------------------------------

template <class Vector>
Vector eval_g(const Vector&  x)
{   Vector g(2);

    g[0] = x[0] * x[1] * x[2] * x[3];
    g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];

    return g;
}

template <class Vector>
Vector eval_jac_g(const Vector&  x)
{   Vector jac_g(8);

    // g[0] = x[0] * x[1] * x[2] * x[3];
    jac_g[0] = x[1] * x[2] * x[3];
    jac_g[1] = x[0] * x[2] * x[3];
    jac_g[2] = x[0] * x[1] * x[3];
    jac_g[3] = x[0] * x[1] * x[2];

    // g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
    jac_g[4+0] = 2. * x[0];
    jac_g[4+1] = 2. * x[1];
    jac_g[4+2] = 2. * x[2];
    jac_g[4+3] = 2. * x[3];

    return jac_g;
}


} // End empty namespace

bool jacobian(void)
{   bool ok = true;
    using CppAD::vector;
    size_t i, j, k;
    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t n = 4;
    size_t m = 2;
    vector< CppAD::AD<double> > ad_x(n);
    vector< CppAD::AD<double> > ad_g(m);

    vector<double> x(n);
    x[0] = 1.; x[1] = 5.0; x[2] = 5.0; x[3] = 1.0;
    for(j = 0; j < n; j++)
        ad_x[j] = x[j];
    //
    CppAD::Independent(ad_x);
    ad_g = eval_g(ad_x);
    CppAD::ADFun<double> fun_g(ad_x, ad_g);

    vector<double> check(m * n);
    check = eval_jac_g(x);

    // regular jacobian
    vector<double> jac_g = fun_g.Jacobian(x);
    for(k = 0; k < m *n; k++)
        ok &= NearEqual(jac_g[k], check[k], eps99, eps99);

    // one argument sparse jacobian
    jac_g = fun_g.SparseJacobian(x);
    for(k = 0; k < m *n; k++)
        ok &= NearEqual(jac_g[k], check[k], eps99, eps99);

    // sparse jacobian using bool vectors
    CPPAD_TESTVECTOR(bool) p_b(m * n) , r_b(n * n);
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            r_b[i * n + j] = (i == j);
    p_b = fun_g.ForSparseJac(n, r_b);
    jac_g = fun_g.SparseJacobian(x, p_b);
    for(k = 0; k < m *n; k++)
        ok &= NearEqual(jac_g[k], check[k], eps99, eps99);

    // sparse jacobian using vectors of sets
    std::vector< std::set<size_t> > p_s(m) , r_s(n);
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            r_s[i].insert(j);
    p_s = fun_g.ForSparseJac(n, r_s);
    jac_g = fun_g.SparseJacobian(x, p_s);
    for(k = 0; k < m *n; k++)
        ok &= NearEqual(jac_g[k], check[k], eps99, eps99);

    return ok;
}
// END PROGRAM
