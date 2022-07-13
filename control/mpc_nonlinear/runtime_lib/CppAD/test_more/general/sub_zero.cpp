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
Test the use of the special parameters zero and one with the multiply operator
*/

# include <cppad/cppad.hpp>

typedef CppAD::AD<double>      ADdouble;
typedef CppAD::AD< ADdouble > ADDdouble;

bool SubZero(void)
{
    using namespace CppAD;

    bool ok = true;

    size_t i;
    for(i = 0; i < 2; i++)
    {   // run through the cases x = 0, 1

        size_t j;
        for(j = 0; j < 2; j++)
        {   // run through the cases y = 0, 1

            CPPAD_TESTVECTOR( ADdouble ) x(1);
            x[0] = double(i);
            Independent(x);

            CPPAD_TESTVECTOR( ADDdouble ) y(1);
            y[0] = ADDdouble(j);
            Independent(y);

            CPPAD_TESTVECTOR( ADDdouble ) z(2);
            z[0]  = x[0] - y[0];
            z[1]  = y[0] - x[0];
            z[1] -= x[0];

            // f(y) = z = { x - y , y - x - x }
            ADFun< ADdouble > f(y, z);
            CPPAD_TESTVECTOR( ADdouble ) u( f.Domain() );
            CPPAD_TESTVECTOR( ADdouble ) v( f.Range() );

            // v = f(y)
            u[0] = ADdouble(j);
            v = f.Forward(0, u);

            // check value of f
            ok &= v[0] == x[0] - ADdouble(j);
            ok &= v[1] == ADdouble(j) - x[0] - x[0];

            // g(x) = f(y) = {x - y , y - x - x}
            ADFun<double> g(x, v);
            CPPAD_TESTVECTOR( double ) a( g.Domain() );
            CPPAD_TESTVECTOR( double ) b( g.Range() );

            // b = g'(x)
            a[0] = 1.;
            b = g.Forward(1, a);

            // check derivatives of g
            ok &= (b[0] == 1.);
            ok &= (b[1] == -2.);

        }
    }

    return ok;
}
