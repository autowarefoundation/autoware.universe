/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin ode_gear.cpp$$
$spell
    Rosen
$$

$section OdeGear: Example and Test$$


Define
$latex x : \B{R} \rightarrow \B{R}^n$$ by
$latex \[
    x_i (t) =  t^{i+1}
\] $$
for $latex i = 1 , \ldots , n-1$$.
It follows that
$latex \[
\begin{array}{rclr}
x_i(0)     & = & 0                             & {\rm for \; all \;} i \\
x_i ' (t)  & = & 1                             & {\rm if \;} i = 0      \\
x_i '(t)   & = & (i+1) t^i = (i+1) x_{i-1} (t) & {\rm if \;} i > 0
\end{array}
\] $$
The example tests OdeGear using the relations above:

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/utility/ode_gear.hpp>
# include <cppad/cppad.hpp>        // For automatic differentiation

namespace {
    class Fun {
    public:
        // constructor
        Fun(bool use_x_) : use_x(use_x_)
        { }

        // compute f(t, x) both for double and AD<double>
        template <class Scalar>
        void Ode(
            const Scalar                    &t,
            const CPPAD_TESTVECTOR(Scalar) &x,
            CPPAD_TESTVECTOR(Scalar)       &f)
        {   size_t n  = x.size();
            Scalar ti(1);
            f[0]   = Scalar(1);
            size_t i;
            for(i = 1; i < n; i++)
            {   ti *= t;
                // convert int(size_t) to avoid warning
                // on _MSC_VER systems
                if( use_x )
                    f[i] = int(i+1) * x[i-1];
                else
                    f[i] = int(i+1) * ti;
            }
        }

        void Ode_dep(
            const double                    &t,
            const CPPAD_TESTVECTOR(double) &x,
            CPPAD_TESTVECTOR(double)       &f_x)
        {   using namespace CppAD;

            size_t n  = x.size();
            CPPAD_TESTVECTOR(AD<double>) T(1);
            CPPAD_TESTVECTOR(AD<double>) X(n);
            CPPAD_TESTVECTOR(AD<double>) F(n);

            // set argument values
            T[0] = t;
            size_t i, j;
            for(i = 0; i < n; i++)
                X[i] = x[i];

            // declare independent variables
            Independent(X);

            // compute f(t, x)
            this->Ode(T[0], X, F);

            // define AD function object
            ADFun<double> fun(X, F);

            // compute partial of f w.r.t x
            CPPAD_TESTVECTOR(double) dx(n);
            CPPAD_TESTVECTOR(double) df(n);
            for(j = 0; j < n; j++)
                dx[j] = 0.;
            for(j = 0; j < n; j++)
            {   dx[j] = 1.;
                df = fun.Forward(1, dx);
                for(i = 0; i < n; i++)
                    f_x [i * n + j] = df[i];
                dx[j] = 0.;
            }
        }

    private:
        const bool use_x;

    };
}

bool OdeGear(void)
{   bool ok = true; // initial return value
    size_t i, j;    // temporary indices
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t  m = 4;  // index of next value in X
    size_t  n = m;  // number of components in x(t)

    // vector of times
    CPPAD_TESTVECTOR(double) T(m+1);
    double step = .1;
    T[0]        = 0.;
    for(j = 1; j <= m; j++)
    {   T[j] = T[j-1] + step;
        step = 2. * step;
    }

    // initial values for x( T[m-j] )
    CPPAD_TESTVECTOR(double) X((m+1) * n);
    for(j = 0; j < m; j++)
    {   double ti = T[j];
        for(i = 0; i < n; i++)
        {   X[ j * n + i ] = ti;
            ti *= T[j];
        }
    }

    // error bound
    CPPAD_TESTVECTOR(double) e(n);

    size_t use_x;
    for( use_x = 0; use_x < 2; use_x++)
    {   // function object depends on value of use_x
        Fun F(use_x > 0);

        // compute OdeGear approximation for x( T[m] )
        CppAD::OdeGear(F, m, n, T, X, e);

        double check = T[m];
        for(i = 0; i < n; i++)
        {   // method is exact up to order m and x[i] = t^{i+1}
            if( i + 1 <= m ) ok &= CppAD::NearEqual(
                X[m * n + i], check, eps99, eps99
            );
            // error bound should be zero up to order m-1
            if( i + 1 < m ) ok &= CppAD::NearEqual(
                e[i], 0., eps99, eps99
            );
            // check value for next i
            check *= T[m];
        }
    }
    return ok;
}

// END C++
