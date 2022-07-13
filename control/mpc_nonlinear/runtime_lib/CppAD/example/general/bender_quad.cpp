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
$begin bender_quad.cpp$$
$spell
    argmin
$$

$section BenderQuad: Example and Test$$


Define
$latex F : \B{R} \times \B{R} \rightarrow \B{R}$$ by
$latex \[
F(x, y)
=
\frac{1}{2} \sum_{i=1}^N [ y * \sin ( x * t_i ) - z_i ]^2
\] $$
where $latex z \in \B{R}^N$$ is a fixed vector.
It follows that
$latex \[
\begin{array}{rcl}
\partial_y F(x, y)
& = &
\sum_{i=1}^N [ y * \sin ( x * t_i ) - z_i ] \sin( x * t_i )
\\
\partial_y \partial_y F(x, y)
& = &
\sum_{i=1}^N \sin ( x t_i )^2
\end{array}
\] $$
Furthermore if we define $latex Y(x)$$
as the argmin of $latex F(x, y)$$ with respect to $latex y$$,
$latex \[
\begin{array}{rcl}
Y(x)
& = &
y - [ \partial_y \partial_y F(x, y) ]^{-1} \partial_y F[x,  y]
\\
& = &
\left.
    \sum_{i=1}^N z_i \sin ( x t_i )
        \right/
            \sum_{i=1}^N z_i \sin ( x * t_i )^2
\end{array}
\] $$



$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    using CppAD::AD;
    typedef CPPAD_TESTVECTOR(double)         BAvector;
    typedef CPPAD_TESTVECTOR(AD<double>)   ADvector;

    class Fun {
    private:
        BAvector t_; // measurement times
        BAvector z_; // measurement values
    public:
        // constructor
        Fun(const BAvector &t, const BAvector &z)
        : t_(t), z_(z)
        { }
        // Fun.f(x, y) = F(x, y)
        ADvector f(const ADvector &x, const ADvector &y)
        {   size_t i;
            size_t N = size_t(z_.size());

            ADvector F(1);
            F[0] = 0.;

            AD<double> residual;
            for(i = 0; i < N; i++)
            {   residual = y[0] * sin( x[0] * t_[i] ) - z_[i];
                F[0]    += .5 * residual * residual;
            }
            return F;
        }
        // Fun.h(x, y) = H(x, y) = F_y (x, y)
        ADvector h(const ADvector &x, const BAvector &y)
        {   size_t i;
            size_t N = size_t(z_.size());

            ADvector fy(1);
            fy[0] = 0.;

            AD<double> residual;
            for(i = 0; i < N; i++)
            {   residual = y[0] * sin( x[0] * t_[i] ) - z_[i];
                fy[0]   += residual * sin( x[0] * t_[i] );
            }
            return fy;
        }
        // Fun.dy(x, y, h) = - H_y (x,y)^{-1} * h
        //                 = - F_yy (x, y)^{-1} * h
        ADvector dy(
            const BAvector &x ,
            const BAvector &y ,
            const ADvector &H )
        {   size_t i;
            size_t N = size_t(z_.size());

            ADvector Dy(1);
            AD<double> fyy = 0.;

            for(i = 0; i < N; i++)
            {   fyy += sin( x[0] * t_[i] ) * sin( x[0] * t_[i] );
            }
            Dy[0] = - H[0] / fyy;

            return Dy;
        }
    };

    // Used to test calculation of Hessian of G
    AD<double> G(const ADvector& x, const BAvector& t, const BAvector& z)
    {   // compute Y(x)
        AD<double> numerator = 0.;
        AD<double> denominator = 0.;
        size_t k;
        for(k = 0; k < size_t(t.size()); k++)
        {   numerator   += sin( x[0] * t[k] ) * z[k];
            denominator += sin( x[0] * t[k] ) * sin( x[0] * t[k] );
        }
        AD<double> y = numerator / denominator;

        // V(x) = F[x, Y(x)]
        AD<double> sum = 0;
        for(k = 0; k < size_t(t.size()); k++)
        {   AD<double> residual = y * sin( x[0] * t[k] ) - z[k];
            sum += .5 * residual * residual;
        }
        return sum;
    }
}

bool BenderQuad(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;

    // temporary indices
    size_t i, j;

    // x space vector
    size_t n = 1;
    BAvector x(n);
    x[0] = 2. * 3.141592653;

    // y space vector
    size_t m = 1;
    BAvector y(m);
    y[0] = 1.;

    // t and z vectors
    size_t N = 10;
    BAvector t(N);
    BAvector z(N);
    for(i = 0; i < N; i++)
    {   t[i] = double(i) / double(N);       // time of measurement
        z[i] = y[0] * sin( x[0] * t[i] );   // data without noise
    }

    // construct the function object
    Fun fun(t, z);

    // evaluate the G(x), G'(x) and G''(x)
    BAvector g(1), gx(n), gxx(n * n);
    CppAD::BenderQuad(x, y, fun, g, gx, gxx);


    // create ADFun object Gfun corresponding to G(x)
    ADvector a_x(n), a_g(1);
    for(j = 0; j < n; j++)
        a_x[j] = x[j];
    Independent(a_x);
    a_g[0] = G(a_x, t, z);
    CppAD::ADFun<double> Gfun(a_x, a_g);

    // accuracy for checks
    double eps = 10. * CppAD::numeric_limits<double>::epsilon();

    // check Jacobian
    BAvector check_gx = Gfun.Jacobian(x);
    for(j = 0; j < n; j++)
        ok &= NearEqual(gx[j], check_gx[j], eps, eps);

    // check Hessian
    BAvector check_gxx = Gfun.Hessian(x, 0);
    for(j = 0; j < n*n; j++)
        ok &= NearEqual(gxx[j], check_gxx[j], eps, eps);

    return ok;
}

// END C++
