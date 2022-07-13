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

void forward_sparse_jacobian_bool(CppAD::ADFun<double>& f)
{   size_t n = f.Domain();
    CPPAD_TESTVECTOR(bool) eye(n * n);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n; j++)
            eye[ i * n + j] = (i == j);
    }
    f.ForSparseJac(n, eye);
    return;
}
void forward_sparse_jacobian_set(CppAD::ADFun<double>& f)
{   size_t n = f.Domain();
    CPPAD_TESTVECTOR(std::set<size_t>) eye(n);
    for(size_t i = 0; i < n; i++)
        eye[i].insert(i);
    f.ForSparseJac(n, eye);
    return;
}

bool sparse_hessian_test(
    CppAD::ADFun<double>&    f     ,
    size_t                   index ,
    CPPAD_TESTVECTOR(bool)&  check )
{   bool   ok = true;
    size_t n  = f.Domain();
    size_t m  = f.Range();

    // boolean sparsity patterns
    CPPAD_TESTVECTOR(bool) r_bool(n), s_bool(m), h_bool(n * n);
    for(size_t j = 0; j < n; j++)
        r_bool[j] = true;
    for(size_t i = 0; i < m; i++)
        s_bool[i] = i == index;
    //
    // bool ForSparseHes
    h_bool = f.ForSparseHes(r_bool, s_bool);
    for(size_t i = 0; i < n * n; i++)
        ok &= h_bool[i] == check[i];
    //
    // bool RevSparseHes
    forward_sparse_jacobian_bool(f);
    h_bool = f.RevSparseHes(n, s_bool);
    for(size_t i = 0; i < n * n; i++)
        ok &= h_bool[i] == check[i];
    //
    // set sparsity patterns
    CPPAD_TESTVECTOR( std::set<size_t> ) r_set(1), s_set(1), h_set(n);
    for(size_t j = 0; j < n; j++)
        r_set[0].insert(j);
    s_set[0].insert(index);
    //
    // set ForSparseHes
    h_set = f.ForSparseHes(r_set, s_set);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n; j++)
        {   bool found = h_set[i].find(j) != h_set[i].end();
            ok        &= found == check[i * n + j];
        }
    }
    //
    // set RevSparseHes
    forward_sparse_jacobian_set(f);
    h_set = f.RevSparseHes(n, s_set);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n; j++)
        {   bool found = h_set[i].find(j) != h_set[i].end();
            ok        &= found == check[i * n + j];
        }
    }
    //
    return ok;
}



bool case_one()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 10;

    // dimension of the range space
    size_t m = 2;

    // temporary indices

    // initialize check values to false
    CPPAD_TESTVECTOR(bool) Check(n * n);
    for(size_t j = 0; j < n * n; j++)
        Check[j] = false;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    for(size_t j = 0; j < n; j++)
        X[j] = AD<double>(j);
    Independent(X);

    // accumulate sum here
    AD<double> sum(0.);

    // variable * variable
    sum += X[2] * X[3];
    Check[2 * n + 3] = Check[3 * n + 2] = true;

    // variable / variable
    sum += X[4] / X[5];
    Check[4 * n + 5] = Check[5 * n + 4] = Check[5 * n + 5] = true;

    // CondExpLt(variable, variable, variable, variable)
    sum += CondExpLt(X[1], X[2], sin(X[6]), cos(X[7]) );
    Check[6 * n + 6] = true;
    Check[7 * n + 7] = true;

    // pow(variable, variable)
    sum += pow(X[8], X[9]);
    Check[8 * n + 8] = Check[8 * n + 9] = true;
    Check[9 * n + 8] = Check[9 * n + 9] = true;

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = sum;

    // variable - variable
    Y[1]  = X[0] - X[1];

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // check Hessian of F_0
    ok &= sparse_hessian_test(F, 0, Check);

    // check Hessian of F_1
    for(size_t j = 0; j < n * n; j++)
        Check[j] = false;
    ok &= sparse_hessian_test(F, 1, Check);

    // -----------------------------------------------------------------------
    return ok;
}

bool case_two()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 4;

    // dimension of the range space
    size_t m = 1;

    // initialize check values to false
    CPPAD_TESTVECTOR(bool) Check(n * n);
    for(size_t j = 0; j < n * n; j++)
        Check[j] = false;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    for(size_t j = 0; j < n; j++)
        X[j] = AD<double>(j);
    Independent(X);

    // Test the case where dependent variable is a non-linear function
    // of the result of a conditional expression.
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = CondExpLt(X[0], X[1], X[2], X[3]);
    Y[0] = cos(Y[0]) + X[0] + X[1];

    // Hessian with respect to x[0] and x[1] is zero.
    // Hessian with respect to x[2] and x[3] is full
    // (although we know that there are no cross terms, this is an
    // inefficiency of the conditional expression operator).
    Check[2 * n + 2] = Check[ 2 * n + 3 ] = true;
    Check[3 * n + 2] = Check[ 3 * n + 3 ] = true;

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // -----------------------------------------------------------------
    sparse_hessian_test(F, 0, Check);
    //
    return ok;
}

bool case_three()
{   bool ok = true;
    using CppAD::AD;

    // domain space vector
    size_t n = 1;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;

    // declare independent variables and start recording
    CppAD::Independent(X);

    // range space vector
    size_t m = 1;
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // make sure reverse jacobian is propagating dependency to
    // intermediate values (not just final ones).
    Y[0] = X[0] * X[0] + 2;

    // create f: X -> Y and stop tape recording
    CppAD::ADFun<double> f(X, Y);

    // ------------------------------------------------------------------
    CPPAD_TESTVECTOR(bool) check(n * n);
    check[0] = true;
    sparse_hessian_test(f, 0, check);
    //
    return ok;
}

bool case_four()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 3;

    // dimension of the range space
    size_t m = 1;

    // inialize the vector as zero
    CppAD::VecAD<double> Z(n - 1);
    size_t k;
    for(k = 0; k < n-1; k++)
        Z[k] = 0.;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;
    X[1] = 1.;
    X[2] = 2.;
    Independent(X);

    // VecAD vector z depends on both x[1] and x[2]
    // (component indices do not matter because they can change).
    Z[ X[0] ] = X[1] * X[2];
    Z[ X[1] ] = 0.;

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // check results vector
    CPPAD_TESTVECTOR( bool )       Check(n * n);

    // y = z[j] where j might be zero or one.
    Y[0]  =  Z[ X[1] ];

    Check[0 * n + 0] = false; // partial w.r.t x[0], x[0]
    Check[0 * n + 1] = false; // partial w.r.t x[0], x[1]
    Check[0 * n + 2] = false; // partial w.r.t x[0], x[2]

    Check[1 * n + 0] = false; // partial w.r.t x[1], x[0]
    Check[1 * n + 1] = false; // partial w.r.t x[1], x[1]
    Check[1 * n + 2] = true;  // partial w.r.t x[1], x[2]

    Check[2 * n + 0] = false; // partial w.r.t x[2], x[0]
    Check[2 * n + 1] = true;  // partial w.r.t x[2], x[1]
    Check[2 * n + 2] = false; // partial w.r.t x[2], x[2]

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // -----------------------------------------------------
    sparse_hessian_test(F, 0, Check);
    //
    return ok;
}

bool case_five(void)
{   bool ok = true;
    using CppAD::AD;

    size_t n = 2;
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 1.;
    X[1] = 2.;
    CppAD::Independent(X);

    size_t m = 2;
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0] = pow(X[0], 2.);
    Y[1] = pow(2., X[1]);

    // create function object F : X -> Y
    CppAD::ADFun<double> F(X, Y);

    // Test F_0 and F_1
    for(size_t index = 0; index < n; index++)
    {   CPPAD_TESTVECTOR(bool) check(n * n);
        for(size_t i = 0; i < n; i++)
            for(size_t j = 0; j < n; j++)
                check[i * n + j] = (i == index) & (j == index);
        sparse_hessian_test(F, index, check);
    }
    //
    return ok;
}

// Note ForSparseHes does not work for this case because R not diagonal.
bool case_six()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 3;

    // dimension of the range space
    size_t m = 1;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 0.;
    X[1] = 1.;
    X[2] = 2.;
    Independent(X);
    // y = z[j] where j might be zero or one.
    CPPAD_TESTVECTOR(AD<double>) Y(m);
    Y[0]  =  X[1] * X[2];
    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // sparsity pattern for hessian of F^2
    CPPAD_TESTVECTOR(bool) F2(n * n);
    F2[0 * n + 0] = false; // partial w.r.t x[0], x[0]
    F2[0 * n + 1] = false; // partial w.r.t x[0], x[1]
    F2[0 * n + 2] = false; // partial w.r.t x[0], x[2]

    F2[1 * n + 0] = false; // partial w.r.t x[1], x[0]
    F2[1 * n + 1] = false; // partial w.r.t x[1], x[1]
    F2[1 * n + 2] = true;  // partial w.r.t x[1], x[2]

    F2[2 * n + 0] = false; // partial w.r.t x[2], x[0]
    F2[2 * n + 1] = true;  // partial w.r.t x[2], x[1]
    F2[2 * n + 2] = false; // partial w.r.t x[2], x[2]

    // choose a non-symmetric sparsity pattern for R
    CPPAD_TESTVECTOR( bool ) r(n * n);
    size_t i, j, k;
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            r[ i * n + j ] = false;
        j = n - i - 1;
        r[ j * n + j ] = true;
    }

    // sparsity pattern for H^T
    CPPAD_TESTVECTOR(bool) Check(n * n);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   Check[ i * n + j] = false;
            for(k = 0; k < n; k++)
            {   // some gcc versions std::vector<bool> do not support |=
                // on elements (because they pack the bits).
                bool tmp         = Check[i * n + j];
                Check[i * n + j] = tmp | (F2[i * n + k] & r[ k * n + j]);
            }
        }
    }

    // compute the reverse Hessian sparsity pattern for F^2 * R
    F.ForSparseJac(n, r);
    CPPAD_TESTVECTOR( bool ) s(m), h(n * n);
    s[0] = 1.;
    bool transpose = true;
    h = F.RevSparseHes(n, s, transpose);

    // check values
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            ok &= (h[i * n + j] == Check[i * n + j]);
    }

    // compute the reverse Hessian sparsity pattern for R^T * F^2
    transpose = false;
    h = F.RevSparseHes(n, s, transpose);

    // check values
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            ok &= (h[j * n + i] == Check[i * n + j]);
    }

    return ok;
}

} // End of empty namespace

bool hes_sparsity(void)
{   bool ok = true;

    ok &= case_one();
    ok &= case_two();
    ok &= case_three();
    ok &= case_four();
    ok &= case_five();
    ok &= case_six();

    return ok;
}
