/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
Test of CondExp with AD< AD< Base > > types
*/

# include <cppad/cppad.hpp>

typedef CppAD::AD< double >     ADdouble;
typedef CppAD::AD< ADdouble > ADADdouble;

namespace { // BEGIN empty namespace

bool CondExpADOne(void)
{   bool ok = true;

    using namespace CppAD;
    size_t n = 3;
    size_t m = 8;

    // ADdouble independent variable vector
    CPPAD_TESTVECTOR( ADdouble ) Xa(n);
    Xa[0] = -1.;
    Xa[1] =  0.;
    Xa[2] =  1.;
    Independent(Xa);

    // ADdouble independent variable vector
    CPPAD_TESTVECTOR( ADADdouble ) Xaa(n);
    Xaa[0] = Xa[0];
    Xaa[1] = Xa[1];
    Xaa[2] = Xa[2];
    Independent(Xaa);

    // ADADdouble parameter
    ADADdouble p = ADADdouble(Xa[0]);
    ADADdouble q = ADADdouble(Xa[1]);
    ADADdouble r = ADADdouble(Xa[2]);

    // ADADdouble dependent variable vector
    CPPAD_TESTVECTOR( ADADdouble ) Yaa(m);

    // CondExp(parameter, parameter, parameter)
    Yaa[0] = CondExp(p, q, r);

    // CondExp(parameter, parameter, variable)
    Yaa[1] = CondExp(p, q, Xaa[2]);

    // CondExp(parameter, varaible, parameter)
    Yaa[2] = CondExp(p, Xaa[1], r);

    // CondExp(parameter, variable, variable)
    Yaa[3] = CondExp(p, Xaa[1], Xaa[2]);

    // CondExp(variable, variable, variable)
    Yaa[5] = CondExp(Xaa[0], Xaa[1], Xaa[2]);

    // CondExp(variable, variable, parameter)
    Yaa[4] = CondExp(Xaa[0], Xaa[1], r);

    // CondExp(variable, parameter, variable)
    Yaa[6] =  CondExp(Xaa[0], q, Xaa[2]);

    // CondExp(variable, parameter, parameter)
    Yaa[7] =  CondExp(Xaa[0], q, r);

    // create fa: Xaa -> Yaa function object
    ADFun< ADdouble > fa(Xaa, Yaa);

    // function values
    CPPAD_TESTVECTOR( ADdouble ) Ya(m);
    Ya  = fa.Forward(0, Xa);

    // create f: Xa -> Ya function object
    ADFun<double> f(Xa, Ya);

    // check result of function evaluation
    CPPAD_TESTVECTOR(double) x(n);
    CPPAD_TESTVECTOR(double) y(m);
    x[0] = 1.;
    x[1] = 0.;
    x[2] = -1.;
    y = f.Forward(0, x);
    size_t i;
    for(i = 0; i < m; i++)
    {   // y[i] = CondExp(x[0], x[1], x[2])
        if( x[0] > 0 )
            ok &= (y[i] == x[1]);
        else
            ok &= (y[i] == x[2]);
    }

    // check forward mode derivatives
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dx[1] = 2.;
    dx[2] = 3.;
    dy    = f.Forward(1, dx);
    for(i = 0; i < m; i++)
    {   if( x[0] > 0. )
            ok &= (dy[i] == dx[1]);
        else
            ok &= (dy[i] == dx[2]);
    }

    // calculate Jacobian
    CPPAD_TESTVECTOR(double) J(m * n);
    size_t j;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            J[i * n + j] = 0.;
        if( x[0] > 0. )
            J[i * n + 1] = 1.;
        else
            J[i * n + 2] = 1.;
    }

    // check reverse mode derivatives
    for(i = 0; i < m; i++)
        dy[i] = double(i);
    dx    = f.Reverse(1, dy);
    double sum;
    for(j = 0; j < n; j++)
    {   sum = 0;
        for(i = 0; i < m; i++)
            sum += dy[i] * J[i * n + j];
        ok &= (sum == dx[j]);
    }

    // forward mode computation of sparsity pattern
    CPPAD_TESTVECTOR(bool) Px(n * n);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            Px[i * n + j] = false;
        Px[i * n + i] = true;
    }
    CPPAD_TESTVECTOR(bool) Py(m * n);
    Py = f.ForSparseJac(n, Px);
    for(i = 0; i < m; i++)
    {   ok &= Py[ i * n + 0 ] == false;
        ok &= Py[ i * n + 1 ] == true;
        ok &= Py[ i * n + 2 ] == true;
    }

    // reverse mode computation of sparsity pattern
    Py.resize(m * m);
    for(i = 0; i < m; i++)
    {   for(j = 0; j < m; j++)
            Py[i * m + j] = false;
        Py[i * m + i] = true;
    }
    Px.resize(m * n);
    Px = f.RevSparseJac(m, Py);
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= ( Px[i * n + j] == ( j > 0 ) );
    }

    return ok;
}
bool CondExpADTwo(void)
{   bool ok = true;

    using namespace CppAD;
    size_t n = 3;
    size_t m = 8;

    // ADdouble independent variable vector
    CPPAD_TESTVECTOR( ADdouble ) Xa(n);
    Xa[0] = -1.;
    Xa[1] =  0.;
    Xa[2] =  1.;
    Independent(Xa);

    // use VecAD so that sparsity results are local
    VecAD<double> Va(1);
    ADdouble zero = 0.;
    Va[zero]      = Xa[0];

    // ADdouble independent variable vector
    CPPAD_TESTVECTOR( ADADdouble ) Xaa(n);
    Xaa[0] = ADdouble( Va[zero] );
    Xaa[1] = Xa[1];
    Xaa[2] = Xa[2];
    Independent(Xaa);

    // ADADdouble parameter
    ADADdouble p = ADADdouble(Xa[0]);
    ADADdouble q = ADADdouble(Xa[1]);
    ADADdouble r = ADADdouble(Xa[2]);

    // ADADdouble dependent variable vector
    CPPAD_TESTVECTOR( ADADdouble ) Yaa(m);

    // CondExp(parameter, parameter, parameter)
    Yaa[0] = CondExp(p, q, r);

    // CondExp(parameter, parameter, variable)
    Yaa[1] = CondExp(p, q, Xaa[2]);

    // CondExp(parameter, varaible, parameter)
    Yaa[2] = CondExp(p, Xaa[1], r);

    // CondExp(parameter, variable, variable)
    Yaa[3] = CondExp(p, Xaa[1], Xaa[2]);

    // CondExp(variable, variable, variable)
    Yaa[5] = CondExp(Xaa[0], Xaa[1], Xaa[2]);

    // CondExp(variable, variable, parameter)
    Yaa[4] = CondExp(Xaa[0], Xaa[1], r);

    // CondExp(variable, parameter, variable)
    Yaa[6] =  CondExp(Xaa[0], q, Xaa[2]);

    // CondExp(variable, parameter, parameter)
    Yaa[7] =  CondExp(Xaa[0], q, r);

    // create fa: Xaa -> Yaa function object
    ADFun< ADdouble > fa(Xaa, Yaa);

    // function values
    CPPAD_TESTVECTOR( ADdouble ) Ya(m);
    Ya  = fa.Forward(0, Xa);

    // create f: Xa -> Ya function object
    ADFun<double> f(Xa, Ya);

    // check use_VecAD
    ok &= f.use_VecAD();

    // check result of function evaluation
    CPPAD_TESTVECTOR(double) x(n);
    CPPAD_TESTVECTOR(double) y(m);
    x[0] = 1.;
    x[1] = 0.;
    x[2] = -1.;
    y = f.Forward(0, x);
    size_t i;
    for(i = 0; i < m; i++)
    {   // y[i] = CondExp(x[0], x[1], x[2])
        if( x[0] > 0 )
            ok &= (y[i] == x[1]);
        else
            ok &= (y[i] == x[2]);
    }

    // check forward mode derivatives
    CPPAD_TESTVECTOR(double) dx(n);
    CPPAD_TESTVECTOR(double) dy(m);
    dx[0] = 1.;
    dx[1] = 2.;
    dx[2] = 3.;
    dy    = f.Forward(1, dx);
    for(i = 0; i < m; i++)
    {   if( x[0] > 0. )
            ok &= (dy[i] == dx[1]);
        else
            ok &= (dy[i] == dx[2]);
    }

    // calculate Jacobian
    CPPAD_TESTVECTOR(double) J(m * n);
    size_t j;
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            J[i * n + j] = 0.;
        if( x[0] > 0. )
            J[i * n + 1] = 1.;
        else
            J[i * n + 2] = 1.;
    }

    // check reverse mode derivatives
    for(i = 0; i < m; i++)
        dy[i] = double(i);
    dx    = f.Reverse(1, dy);
    double sum;
    for(j = 0; j < n; j++)
    {   sum = 0;
        for(i = 0; i < m; i++)
            sum += dy[i] * J[i * n + j];
        ok &= (sum == dx[j]);
    }

    // forward mode computation of sparsity pattern
    CPPAD_TESTVECTOR(bool) Px(n * n);
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
            Px[i * n + j] = false;
        Px[i * n + i] = true;
    }
    CPPAD_TESTVECTOR(bool) Py(m * n);
    Py = f.ForSparseJac(n, Px);
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            // sparsity pattern works for both true and false cases.
            ok &= ( Py[i * n + j] == (j > 0) );
    }

    // reverse mode computation of sparsity pattern
    Py.resize(m * m);
    for(i = 0; i < m; i++)
    {   for(j = 0; j < m; j++)
            Py[i * m + j] = false;
        Py[i * m + i] = true;
    }
    Px.resize(m * n);
    Px = f.RevSparseJac(m, Py);
    for(i = 0; i < m; i++)
    {   for(j = 0; j < n; j++)
            ok &= ( Px[i * n + j] == (j > 0) );
    }

    return ok;
}

} // END empty namespace

bool CondExpAD(void)
{   bool ok = true;
    ok     &= CondExpADOne();
    ok     &= CondExpADTwo();
    return ok;
}
