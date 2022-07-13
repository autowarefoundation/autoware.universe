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
# include <vector>
# include <valarray>

# define CheckOp(Op)                   \
    Y[index] = X[0] Op 2.;         \
    Check[index * n + 0] = true;   \
    Check[index * n + 1] = false;  \
    Check[index * n + 2] = false;  \
    index++;                       \
    Y[index] = X[0] Op X[1];       \
    Check[index * n + 0] = true;   \
    Check[index * n + 1] = true;   \
    Check[index * n + 2] = false;  \
    index++;                       \
    Y[index] = 3.   Op X[1];       \
    Check[index * n + 0] = false;  \
    Check[index * n + 1] = true;   \
    Check[index * n + 2] = false;  \
    index++;

# define CheckUnaryFun(Fun)            \
    Y[index] = Fun(X[0]);          \
    Check[index * n + 0] = true;   \
    Check[index * n + 1] = false;  \
    Check[index * n + 2] = false;  \
    index++;                       \
    Y[index] = Fun(X[0] + X[1]);   \
    Check[index * n + 0] = true;   \
    Check[index * n + 1] = true;   \
    Check[index * n + 2] = false;  \
    index++;                       \
    Y[index] = Fun(X[1]);          \
    Check[index * n + 0] = false;  \
    Check[index * n + 1] = true;   \
    Check[index * n + 2] = false;  \
    index++;


# define CheckBinaryFun(Fun)           \
    Y[index] = Fun( X[0] , 2.);    \
    Check[index * n + 0] = true;   \
    Check[index * n + 1] = false;  \
    Check[index * n + 2] = false;  \
    index++;                       \
    Y[index] = Fun( X[0] , X[1]);  \
    Check[index * n + 0] = true;   \
    Check[index * n + 1] = true;   \
    Check[index * n + 2] = false;  \
    index++;                       \
    Y[index] = Fun( 3.   , X[1]);  \
    Check[index * n + 0] = false;  \
    Check[index * n + 1] = true;   \
    Check[index * n + 2] = false;  \
    index++;

namespace { // BEGIN empty namespace

bool case_one()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 3;

    // dimension of the range space
    size_t m = (4 + 11 + 1) * 3 + 4;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = .1;
    X[1] = .2;
    X[2] = .3;
    Independent(X);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // check results vector
    CPPAD_TESTVECTOR( bool )       Check(m * n);

    // initialize index into Y
    size_t index = 0;

    // 4 binary operators
    CheckOp(+);
    CheckOp(-);
    CheckOp(*);
    CheckOp(/);

    // 11 unary functions
    CheckUnaryFun(abs);
    CheckUnaryFun(acos);
    CheckUnaryFun(asin);
    CheckUnaryFun(atan);
    CheckUnaryFun(cos);
    CheckUnaryFun(cosh);
    CheckUnaryFun(exp);
    CheckUnaryFun(log);
    CheckUnaryFun(sin);
    CheckUnaryFun(sinh);
    CheckUnaryFun(sqrt);

    // 1 binary function
    CheckBinaryFun(pow);

    // conditional expression
    Y[index] = CondExpLt(X[0], X[1], X[0], AD<double>(2.));
    Check[index * n + 0] = true;
    Check[index * n + 1] = false;
    Check[index * n + 2] = false;
    index++;
    Y[index] = CondExpLt(X[0], X[1], X[0], X[1]);
    Check[index * n + 0] = true;
    Check[index * n + 1] = true;
    Check[index * n + 2] = false;
    index++;
    Y[index] = CondExpLt(X[0], X[1], AD<double>(3.), X[1]);
    Check[index * n + 0] = false;
    Check[index * n + 1] = true;
    Check[index * n + 2] = false;
    index++;

    // non-trival composition
    Y[index] = Y[0] + Y[1] + X[2];
    Check[index * n + 0] = true;
    Check[index * n + 1] = true;
    Check[index * n + 2] = true;
    index++;

    // check final index
    assert( index == m );


    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // --------------------------------------------------------
    // dependency matrix for the identity function U(y) = y
    CPPAD_TESTVECTOR( bool ) Py(m * m);
    size_t i, j;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < m; j++)
            Py[ i * m + j ] = false;
        Py[ i * m + i ] = true;
    }

    // evaluate the dependency matrix for F(x)
    CPPAD_TESTVECTOR( bool ) Px(m * n);
    Px = F.RevSparseJac(m, Py);

    // check values
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Px[i * n + j] == Check[i * n + j]);
    }
    // --------------------------------------------------------
    // dependency matrix for the identity function U(y) = y
    CPPAD_TESTVECTOR(std::set<size_t>) Sy(m);
    for(i = 0; i < m; i++)
    {   assert( Sy[i].empty() );
        Sy[i].insert(i);
    }

    // evaluate the dependency matrix for U(F(x))
    CPPAD_TESTVECTOR(std::set<size_t>) Sx(m);
    Sx = F.RevSparseJac(m, Sy);

    // check values
    std::set<size_t>::iterator itr;
    bool found;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
        {   found = Sx[i].find(j) != Sx[i].end();
            ok    &= (found == Check[i * n + j]);
        }
    }

    return ok;
}

bool case_two()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 3;

    // dimension of the range space
    size_t m = 3;

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

    // VecAD vector is going to depend on X[1] and X[2]
    Z[ X[0] ] = X[1];
    Z[ X[1] ] = X[2];

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // check results vector
    CPPAD_TESTVECTOR( bool )       Check(m * n);

    // initialize index into Y
    size_t index = 0;

    // First component only depends on X[0];
    Y[index]             = X[0];
    Check[index * n + 0] = true;
    Check[index * n + 1] = false;
    Check[index * n + 2] = false;
    index++;

    // Second component depends on the vector Z
    AD<double> zero(0);
    Y[index]             = Z[zero]; // Load by a parameter
    Check[index * n + 0] = false;
    Check[index * n + 1] = true;
    Check[index * n + 2] = true;
    index++;

    // Third component depends on the vector Z
    Y[index]             = Z[ X[0] ]; // Load by a variable
    Check[index * n + 0] = false;
    Check[index * n + 1] = true;
    Check[index * n + 2] = true;
    index++;

    // check final index
    assert( index == m );

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // dependency matrix for the identity function S(y) = y
    CPPAD_TESTVECTOR( bool ) Py(m * m);
    size_t i, j;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < m; j++)
            Py[ i * m + j ] = false;
        Py[ i * m + i ] = true;
    }

    // evaluate the dependency matrix for S [ F(x) ]
    CPPAD_TESTVECTOR( bool ) Px(m * n);
    Px = F.RevSparseJac(m, Py);

    // check values
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Px[i * n + j] == Check[i * n + j]);
    }
    // --------------------------------------------------------
    // dependency matrix for the identity function U(y) = y
    CPPAD_TESTVECTOR(std::set<size_t>) Sy(m);
    for(i = 0; i < m; i++)
    {   assert( Sy[i].empty() );
        Sy[i].insert(i);
    }

    // evaluate the dependency matrix for U(F(x))
    CPPAD_TESTVECTOR(std::set<size_t>) Sx(m);
    Sx = F.RevSparseJac(m, Sy);

    // check values
    std::set<size_t>::iterator itr;
    bool found;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
        {   found = Sx[i].find(j) != Sx[i].end();
            ok    &= (found == Check[i * n + j]);
        }
    }

    return ok;
}

bool case_three()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 2;

    // dimension of the range space
    size_t m = 3;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 2.;
    X[1] = 3.;
    Independent(X);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // check results vector
    CPPAD_TESTVECTOR( bool )       Check(m * n);

    // initialize index into Y
    size_t index = 0;

    // Y[0] only depends on X[0];
    Y[index]             = pow(X[0], 2.);
    Check[index * n + 0] = true;
    Check[index * n + 1] = false;
    index++;

    // Y[1] depends on X[1]
    Y[index]             = pow(2., X[1]);
    Check[index * n + 0] = false;
    Check[index * n + 1] = true;
    index++;

    // Y[2] depends on X[0] and X[1]
    Y[index]             = pow(X[0], X[1]);
    Check[index * n + 0] = true;
    Check[index * n + 1] = true;
    index++;

    // check final index
    assert( index == m );

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // -----------------------------------------------------------------
    // dependency matrix for the identity function
    CPPAD_TESTVECTOR( bool ) Py(m * m);
    size_t i, j;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < m; j++)
            Py[ i * m + j ] = (i == j);
    }

    // evaluate the dependency matrix for F(x)
    CPPAD_TESTVECTOR( bool ) Px(m * n);
    Px = F.RevSparseJac(m, Py);

    // check values
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Px[i * n + j] == Check[i * n + j]);
    }

    // ---------------------------------------------------------
    // dependency matrix for the identity function
    CPPAD_TESTVECTOR(std::set<size_t>) Sy(m);
    for(i = 0; i < m; i++)
    {   assert( Sy[i].empty() );
        Sy[i].insert(i);
    }

    // evaluate the dependency matrix for F(x)
    CPPAD_TESTVECTOR(std::set<size_t>) Sx(m);
    Sx = F.RevSparseJac(m, Sy);

    // check values
    bool found;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
        {   found = Sx[i].find(j) != Sx[i].end();
            ok &= (found == Check[i * n + j]);
        }
    }

    return ok;
}


// case where s is not identity matrix
bool case_four()
{
    bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 2;

    // dimension of the range space
    size_t m = n;

    // independent and variable vectors
    CPPAD_TESTVECTOR(AD<double>) ax(n), ay(m);
    ax[0] = 2.;
    ax[1] = 3.;
    Independent(ax);
    ay[0] = ax[1];
    ay[1] = ax[0];

    // create function object F : x -> y
    ADFun<double> F(ax, ay);

    // evaluate the dependency matrix for F(x)
    size_t q = 1;
    CPPAD_TESTVECTOR( bool ) s(q * m), r(q * n);
    s[0] = true;
    s[1] = false;
    r = F.RevSparseJac(q, s);

    ok &= size_t( r.size() ) == q * n;
    ok &= r[0] == false;
    ok &= r[1] == true;

    return ok;
}

bool case_five()
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 2;

    // dimension of the range space
    size_t m = 3;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    X[0] = 2.;
    X[1] = 3.;
    Independent(X);

    // dependent variable vector
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // check results vector
    CPPAD_TESTVECTOR( bool )       Check(m * n);

    // initialize index into Y
    size_t index = 0;

    // Y[0] only depends on X[0];
    Y[index]             = pow(X[0], 2.);
    Check[index * n + 0] = true;
    Check[index * n + 1] = false;
    index++;

    // Y[1] depends on X[1]
    Y[index]             = pow(2., X[1]);
    Check[index * n + 0] = false;
    Check[index * n + 1] = true;
    index++;

    // Y[2] depends on X[0] and X[1]
    Y[index]             = pow(X[0], X[1]);
    Check[index * n + 0] = true;
    Check[index * n + 1] = true;
    index++;

    // check final index
    assert( index == m );

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // -----------------------------------------------------------------
    // dependency matrix for the identity function
    CPPAD_TESTVECTOR( bool ) Py(m * m);
    size_t i, j;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < m; j++)
            Py[ i * m + j ] = (i == j);
    }

    // evaluate the dependency matrix for F(x)
    bool transpose = true;
    CPPAD_TESTVECTOR( bool ) Px(n * m);
    Px = F.RevSparseJac(m, Py, transpose);

    // check values
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Px[j * m + i] == Check[i * n + j]);
    }

    // ---------------------------------------------------------
    // dependency matrix for the identity function
    CPPAD_TESTVECTOR(std::set<size_t>) Sy(m);
    for(i = 0; i < m; i++)
    {   assert( Sy[i].empty() );
        Sy[i].insert(i);
    }

    // evaluate the dependency matrix for F(x)
    CPPAD_TESTVECTOR(std::set<size_t>) Sx(n);
    Sx = F.RevSparseJac(m, Sy, transpose);

    // check values
    bool found;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
        {   found = Sx[j].find(i) != Sx[j].end();
            ok &= (found == Check[i * n + j]);
        }
    }

    return ok;
}



} // END empty namespace

bool rev_sparse_jac(void)
{   bool ok = true;

    ok &= case_one();
    ok &= case_two();
    ok &= case_three();
    ok &= case_four();
    ok &= case_five();

    return ok;
}
