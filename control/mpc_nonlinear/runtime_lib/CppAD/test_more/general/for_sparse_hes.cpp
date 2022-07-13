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

namespace { // Begin empty namespace

bool test_one()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 14;

    // dimension of the range space
    size_t m = 1;

    // temporary indices
    size_t i, j;

    // for testing load and store operations
    CppAD::VecAD<double> ad_vec(2);
    ad_vec[0] = 3.0;
    ad_vec[1] = 4.0;

    // initialize check values to false
    CPPAD_TESTVECTOR(bool) check(n * n);
    for(j = 0; j < n * n; j++)
        check[j] = false;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    for(j = 0; j < n; j++)
        ax[j] = AD<double>(j);
    Independent(ax);

    // accumulate sum here
    AD<double> sum(0.);

    // first operand
    size_t F = 0;

    // ad_vec[variable] when ad_vec is a parameter
    sum += ad_vec[ax[F]]; // use fact ax[F] is zero
    F += 1;

    // ad_vec[parameter] when ad_vec depends on a variable
    // (CppAD sparsity does not separate elements of ad_vec)
    ad_vec[ AD<double>(0) ] = ax[F] * ax[F];
    sum += ad_vec[ ax[F] ];  // user fact that ax[F] is one
    check[F * n + F] = true;
    F += 1;

    // parameter / variable
    sum += 2.0 / ax[F];
    check[F * n + F] = true;
    F += 1;

    // erf(variable)
    sum += erf( ax[F] );
    check[F * n + F] = true;
    F += 1;

    // pow(parameter, variable)
    sum += pow( 2.0 , ax[F] );
    check[F * n + F] = true;
    F += 1;

    // pow(variable, parameter)
    sum += pow( ax[F] , 2.0 );
    check[F * n + F] = true;
    F += 1;
    // second operand
    size_t S = F + 1;

    // variable * variable
    sum += ax[F] * ax[S];
    check[F * n + S] = check[S * n + F] = true;
    F += 2;
    S += 2;

    // azmul(variable, variable)
    sum += azmul(ax[F], ax[S]);
    check[F * n + S] = check[S * n + F] = true;
    F += 2;
    S += 2;

    // variable / variable
    sum += ax[F] / ax[S];
    check[F * n + S] = check[S * n + F] = check[S * n + S] = true;
    F += 2;
    S += 2;

    // pow( variable , variable )
    sum += pow( ax[F] , ax[S] );
    check[F * n + F] = check[S * n + S] = true;
    check[F * n + S] = check[S * n + F] = true;
    F += 2;
    S += 2;

    ok &= F == n;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[0] = sum;

    // create function object f : x -> y
    ADFun<double> f(ax, ay);

    // ------------------------------------------------------------------
    // compute sparsity
    CPPAD_TESTVECTOR(bool) r(n), s(m), h(n * n);
    for(j = 0; j < n; j++)
        r[j] = true;
    for(i = 0; i < m; i++)
        s[i] = true;
    h = f.ForSparseHes(r, s);
    // check result
    for(i = 0; i < n; i++)
        for(j = 0; j < n; j++)
            ok &= h[i * n + j] == check[i * n + j];
    // ------------------------------------------------------------------

    return ok;
}

} // End of empty namespace

bool for_sparse_hes(void)
{   bool ok = true;

    ok &= test_one();

    return ok;
}
