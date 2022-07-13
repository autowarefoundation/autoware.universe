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

bool DivZeroOne(void)
{   bool ok = true;
    using namespace CppAD;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t i;
    for(i = 0; i < 3; i++)
    {   // run through the cases x = 0, 1, 2

        size_t j;
        for(j = 0; j < 3; j++)
        {   // run through the cases y = 0, 1, 2

            CPPAD_TESTVECTOR( ADdouble ) x(1);
            x[0] = double(i);
            Independent(x);

            CPPAD_TESTVECTOR( ADDdouble ) y(1);
            y[0] = ADDdouble(j);
            Independent(y);

            CPPAD_TESTVECTOR( ADDdouble ) z(2);
            if( j == 0 )
                z[0] = ADDdouble(0);
            else
                z[0] = x[0] / y[0];
            if( i == 0 )
                z[1] = ADDdouble(0);
            else
            {   z[1]  = y[0] / x[0];
                z[1] /= x[0];
            }

            // f(y) = z = { x / y , y / (x * x) }
            ADFun< ADdouble > f(y, z);
            CPPAD_TESTVECTOR( ADdouble ) u( f.Domain() );
            CPPAD_TESTVECTOR( ADdouble ) v( f.Range() );

            // v = f'(y)
            u[0] = ADdouble(1.);
            v = f.Forward(1, u);

            // check derivatives of f
            ADdouble check = - double(i) / double(j * j);
            if( j != 0 ) ok &= NearEqual(
                v[0], check, eps99, eps99 );

            check = 1. / double(i * i);
            if( i != 0 ) ok &= NearEqual(
                v[1], check, eps99, eps99);

            // g(x) = f'(y) = {-x/y^2 , 1/(x * x)}
            ADFun<double> g(x, v);
            CPPAD_TESTVECTOR( double ) a( g.Domain() );
            CPPAD_TESTVECTOR( double ) b( g.Range() );

            // b = g'(x)
            a[0] = 1.;
            b = g.Forward(1, a);

            // check derivatives of g
            if( j != 0 ) ok &= NearEqual(
                b[0], - 1./double(j*j), eps99, eps99 );
            if( i != 0 ) ok &= NearEqual(
                b[1], -2./double(i*i*i), eps99, eps99);

        }
    }

    return ok;
}
