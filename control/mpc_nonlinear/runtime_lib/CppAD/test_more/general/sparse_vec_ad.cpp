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

bool sparse_vec_ad(void)
{   bool ok = true;
    using namespace CppAD;

    // dimension of the domain space
    size_t n = 3;

    size_t i, j;

    // independent variable vector
    CPPAD_TESTVECTOR(AD<double>) X(n);
    for(j = 0; j < n; j++)
        X[j] = AD<double>(j);
    Independent(X);

    // dependent variable vector
    size_t m = n;
    CPPAD_TESTVECTOR(AD<double>) Y(m);

    // check results vector
    CPPAD_TESTVECTOR( bool )  Check(m * n);

    // Create a VecAD so that there are two in the tape and the sparsity
    // pattern depends on the second one (checks addressing VecAD objects)
    VecAD<double> W(n);

    // VecAD equal to X
    VecAD<double> Z(n);
    AD<double> J;
    for(j = 0; j < n; j++)
    {   J    = AD<double>(j);
        W[J] = X[0];
        Z[J] = X[j];

        // y[i] depends on x[j] for j <= i
        // (and is non-linear for j <= 1).
        if( j == 1 )
            Y[j] = Z[J] * Z[J];
        else
            Y[j] = Z[J];
    }

    // compute dependent variables values
    AD<double> P = 1;
    J = AD<double>(0);
    for(j = 0; j < n; j++)
    {   for(i = 0; i < m; i++)
            Check[ i * m + j ] = (j <= i);
    }

    // create function object F : X -> Y
    ADFun<double> F(X, Y);

    // dependency matrix for the identity function W(x) = x
    CPPAD_TESTVECTOR( bool ) Identity(n * n);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            Identity[ i * n + j ] = false;
        Identity[ i * n + i ] = true;
    }
    // evaluate the dependency matrix for Identity(F(x))
    CPPAD_TESTVECTOR( bool ) Px(m * n);
    Px = F.RevSparseJac(n, Identity);

    // check values
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Px[i * m + j] == Check[i * m + j]);
    }

    // evaluate the dependency matrix for F(Identity(x))
    CPPAD_TESTVECTOR( bool ) Py(m * n);
    Py = F.ForSparseJac(n, Identity);

    // check values
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Py[i * m + j] == Check[i * m + j]);
    }

    // test sparsity pattern for Hessian of F_2 ( Identity(x) )
    CPPAD_TESTVECTOR(bool) Hy(m);
    for(i = 0; i < m; i++)
        Hy[i] = false;
    Hy[2] = true;
    CPPAD_TESTVECTOR(bool) Pxx(n * n);
    Pxx = F.RevSparseHes(n, Hy);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Pxx[i * n + j] == false );
    }

    // test sparsity pattern for Hessian of F_1 ( Identity(x) )
    for(i = 0; i < m; i++)
        Hy[i] = false;
    Hy[1] = true;
    Pxx = F.RevSparseHes(n, Hy);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            ok &= (Pxx[i * n + j] == ( (i <= 1) & (j <= 1) ) );
    }


    return ok;
}
