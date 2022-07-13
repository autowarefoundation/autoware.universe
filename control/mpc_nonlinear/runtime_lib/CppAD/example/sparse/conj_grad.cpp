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
$begin conj_grad.cpp$$
$spell
    checkpointing
    goto
$$

$section Differentiate Conjugate Gradient Algorithm: Example and Test$$


$head Purpose$$
The conjugate gradient algorithm is sparse linear solver and
a good example where checkpointing can be applied (for each iteration).
This example is a preliminary version of a new library routine
for the conjugate gradient algorithm.

$head Algorithm$$
Given a positive definite matrix $latex A \in \B{R}^{n \times n}$$,
a vector $latex b \in \B{R}^n$$,
and tolerance $latex \varepsilon$$,
the conjugate gradient algorithm finds an $latex x \in \B{R}^n$$
such that $latex \| A x - b \|^2 / n \leq \varepsilon^2$$
(or it terminates at a specified maximum number of iterations).

$list number$$
Input:
$pre
$$
The matrix $latex A \in \B{R}^{n \times n}$$,
the vector $latex b \in \B{R}^n$$,
a tolerance $latex \varepsilon \geq 0$$,
a maximum number of iterations $latex m$$,
and the initial approximate solution $latex x^0 \in \B{R}^n$$
(can use zero for $latex x^0$$).

$lnext
Initialize:
$pre
$$
$latex g^0 = A * x^0 - b$$,
$latex d^0 = - g^0$$,
$latex s_0 = ( g^0 )^\R{T} g^0$$,
$latex k = 0$$.

$lnext
Convergence Check:
$pre
$$
if $latex k = m$$ or $latex \sqrt{ s_k / n } < \varepsilon $$,
return $latex k$$ as the number of iterations and $latex x^k$$
as the approximate solution.

$lnext
Next $latex x$$:
$pre
$$
$latex \mu_{k+1} = s_k / [ ( d^k )^\R{T} A d^k ]$$,
$latex x^{k+1} = x^k + \mu_{k+1} d^k$$.

$lnext
Next $latex g$$:
$pre
$$
$latex g^{k+1} = g^k + \mu_{k+1} A d^k$$,
$latex s_{k+1} = ( g^{k+1} )^\R{T} g^{k+1}$$.

$lnext
Next $latex d$$:
$pre
$$
$latex d^{k+1} = - g^k + ( s_{k+1} / s_k ) d^k$$.

$lnext
Iterate:
$pre
$$
$latex k = k + 1$$,
goto Convergence Check.
$lend

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
# include <cstdlib>
# include <cmath>

namespace { // Begin empty namespace
    using CppAD::AD;

    // A simple matrix multiply c = a * b , where a has n columns
    // and b has n rows. This should be changed to a function so that
    // it can efficiently handle the case were A is large and sparse.
    template <class Vector> // a simple vector class
    void mat_mul(size_t n, const Vector& a, const Vector& b, Vector& c)
    {   typedef typename Vector::value_type scalar;

        size_t m, p;
        m = a.size() / n;
        p = b.size() / n;

        assert( m * n == a.size() );
        assert( n * p == b.size() );
        assert( m * p == c.size() );

        size_t i, j, k, ij;
        for(i = 0; i < m; i++)
        {   for(j = 0; j < p; j++)
            {   ij    = i * p + j;
                c[ij] = scalar(0);
                for(k = 0; k < n; k++)
                    c[ij] = c[ij] + a[i * m + k] * b[k * p + j];
            }
        }
        return;
    }

    // Solve A * x == b to tolerance epsilon or terminate at m interations.
    template <class Vector> // a simple vector class
    size_t conjugate_gradient(
        size_t         m       , // input
        double         epsilon , // input
        const Vector&  A       , // input
        const Vector&  b       , // input
        Vector&        x       ) // input / output
    {   typedef typename Vector::value_type scalar;
        scalar mu, s_previous;
        size_t i, k;

        size_t n = x.size();
        assert( A.size() == n * n );
        assert( b.size() == n );

        Vector g(n), d(n), s(1), Ad(n), dAd(1);

        // g = A * x
        mat_mul(n, A, x, g);
        for(i = 0; i < n; i++)
        {   // g = A * x - b
            g[i] = g[i] - b[i];

            // d = - g
            d[i] = -g[i];
        }
        // s = g^T * g
        mat_mul(n, g, g, s);

        for(k = 0; k < m; k++)
        {   s_previous = s[0];
            if( s_previous < epsilon )
                return k;

            // Ad = A * d
            mat_mul(n, A, d, Ad);

            // dAd = d^T * A * d
            mat_mul(n, d, Ad, dAd);

            // mu = s / d^T * A * d
            mu = s_previous / dAd[0];

            // g = g + mu * A * d
            for(i = 0; i < n; i++)
            {   x[i] = x[i] + mu * d[i];
                g[i] = g[i] + mu * Ad[i];
            }

            // s = g^T * g
            mat_mul(n, g, g, s);

            // d = - g + (s / s_previous) * d
            for(i = 0; i < n; i++)
                d[i] = - g[i] + ( s[0] / s_previous) * d[i];
        }
        return m;
    }

} // End empty namespace

bool conj_grad(void)
{   bool ok = true;

    // ----------------------------------------------------------------------
    // Setup
    // ----------------------------------------------------------------------
    using CppAD::AD;
    using CppAD::NearEqual;
    using CppAD::vector;
    using std::cout;
    using std::endl;
    size_t i, j;


    // size of the vectors
    size_t n  = 40;
    vector<double> D(n * n), Dt(n * n), A(n * n), x(n), b(n), c(n);
    vector< AD<double> > a_A(n * n), a_x(n), a_b(n);

    // D = diagonally dominant matrix
    // c = vector of ones
    for(i = 0; i < n; i++)
    {   c[i] = 1.;
        double sum = 0;
        for(j = 0; j < n; j++) if( i != j )
        {   D[ i * n + j ] = std::rand() / double(RAND_MAX);
            Dt[j * n + i ] = D[i * n + j ];
            sum           += D[i * n + j ];
        }
        Dt[ i * n + i ] = D[ i * n + i ] = sum * 1.1;
    }

    // A = D^T * D
    mat_mul(n, Dt, D, A);

    // b = D^T * c
    mat_mul(n, Dt, c, b);

    // copy from double to AD<double>
    for(i = 0; i < n; i++)
    {   a_b[i] = b[i];
        for(j = 0; j < n; j++)
            a_A[ i * n + j ] = A[ i * n + j ];
    }

    // ---------------------------------------------------------------------
    // Record the function f : b -> x
    // ---------------------------------------------------------------------
    // Make b the independent variable vector
    Independent(a_b);

    // Solve A * x = b using conjugate gradient method
    double epsilon = 1e-7;
    for(i = 0; i < n; i++)
            a_x[i] = AD<double>(0);
    size_t m = n + 1;
    size_t k = conjugate_gradient(m, epsilon, a_A, a_b, a_x);

    // create f_cg: b -> x and stop tape recording
    CppAD::ADFun<double> f(a_b, a_x);

    // ---------------------------------------------------------------------
    // Check for correctness
    // ---------------------------------------------------------------------

    // conjugate gradient should converge with in n iterations
    ok &= (k <= n);

    // accuracy to which we expect values to agree
    double delta = 10. * epsilon * std::sqrt( double(n) );

    // copy x from AD<double> to double
    for(i = 0; i < n; i++)
        x[i] = Value( a_x[i] );

    // check c = A * x
    mat_mul(n, A, x, c);
    for(i = 0; i < n; i++)
        ok &= NearEqual(c[i] , b[i],  delta , delta);

    // forward computation of partials w.r.t. b[0]
    vector<double> db(n), dx(n);
    for(j = 0; j < n; j++)
        db[j] = 0.;
    db[0] = 1.;

    // check db = A * dx
    delta = 5. * delta;
    dx = f.Forward(1, db);
    mat_mul(n, A, dx, c);
    for(i = 0; i < n; i++)
        ok   &= NearEqual(c[i], db[i], delta, delta);

    return ok;
}

// END C++
