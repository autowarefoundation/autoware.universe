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
$begin mul_level_ode.cpp$$
$spell
    Taylor
    Cpp
    const
    std
    Adolc
    adouble
$$

$section Taylor's Ode Solver: A Multi-Level AD Example and Test$$

$head See Also$$
$cref taylor_ode.cpp$$, $cref base2ad.cpp$$, $cref mul_level_adolc_ode.cpp$$

$head Purpose$$
This is a realistic example using
two levels of AD; see $cref mul_level$$.
The first level uses $code AD<double>$$ to tape the solution of an
ordinary differential equation.
This solution is then differentiated with respect to a parameter vector.
The second level uses $code AD< AD<double> >$$
to take derivatives during the solution of the differential equation.
These derivatives are used in the application
of Taylor's method to the solution of the ODE.

$head ODE$$
For this example the function
$latex y : \B{R} \times \B{R}^n \rightarrow \B{R}^n$$ is defined by
$latex y(0, x) = 0$$ and
$latex \partial_t y(t, x) = g(y, x)$$ where
$latex g : \B{R}^n \times \B{R}^n \rightarrow \B{R}^n$$ is defined by
$latex \[
    g(y, x) =
    \left( \begin{array}{c}
            x_0     \\
            x_1 y_0 \\
            \vdots  \\
            x_{n-1} y_{n-2}
    \end{array} \right)
\] $$

$head ODE Solution$$
The solution for this example can be calculated by
starting with the first row and then using the solution
for the first row to solve the second and so on.
Doing this we obtain
$latex \[
    y(t, x ) =
    \left( \begin{array}{c}
        x_0 t                  \\
        x_1 x_0 t^2 / 2        \\
        \vdots                 \\
        x_{n-1} x_{n-2} \ldots x_0 t^n / n !
    \end{array} \right)
\] $$

$head Derivative of ODE Solution$$
Differentiating the solution above,
with respect to the parameter vector $latex x$$,
we notice that
$latex \[
\partial_x y(t, x ) =
\left( \begin{array}{cccc}
y_0 (t,x) / x_0      & 0                   & \cdots & 0      \\
y_1 (t,x) / x_0      & y_1 (t,x) / x_1     & 0      & \vdots \\
\vdots               & \vdots              & \ddots & 0      \\
y_{n-1} (t,x) / x_0  & y_{n-1} (t,x) / x_1 & \cdots & y_{n-1} (t,x) / x_{n-1}
\end{array} \right)
\] $$

$head Taylor's Method Using AD$$
We define the function $latex z(t, x)$$ by the equation
$latex \[
    z ( t , x ) = g[ y ( t , x ) ] = h [ x , y( t , x ) ]
\] $$
see $cref taylor_ode$$ for the method used to compute the
Taylor coefficients w.r.t $latex t$$ of $latex y(t, x)$$.

$head Source$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
--------------------------------------------------------------------------
*/
// BEGIN C++

# include <cppad/cppad.hpp>

// =========================================================================
// define types for each level
namespace { // BEGIN empty namespace

typedef CppAD::AD<double>          a1double;
typedef CppAD::AD<a1double>        a2double;

typedef CPPAD_TESTVECTOR(double)    d_vector;
typedef CPPAD_TESTVECTOR(a1double)  a1vector;
typedef CPPAD_TESTVECTOR(a2double)  a2vector;

// -------------------------------------------------------------------------
// class definition for C++ function object that defines ODE
class Ode {
private:
    // copy of a that is set by constructor and used by g(y)
    a1vector a1x_;
public:
    // constructor
    Ode(const a1vector& a1x) : a1x_(a1x)
    { }
    // the function g(y) is evaluated with two levels of taping
    a2vector operator()
    ( const a2vector& a2y) const
    {   size_t n = a2y.size();
        a2vector a2g(n);
        size_t i;
        a2g[0] = a1x_[0];
        for(i = 1; i < n; i++)
            a2g[i] = a1x_[i] * a2y[i-1];

        return a2g;
    }
};

// -------------------------------------------------------------------------
// Routine that uses Taylor's method to solve ordinary differential equaitons
// and allows for algorithmic differentiation of the solution.
a1vector taylor_ode(
    Ode                            G       ,  // function that defines the ODE
    size_t                         order   ,  // order of Taylor's method used
    size_t                         nstep   ,  // number of steps to take
    const a1double&                a1dt    ,  // Delta t for each step
    const a1vector& a1y_ini)  // y(t) at the initial time
{
    // some temporary indices
    size_t i, k, ell;

    // number of variables in the ODE
    size_t n = a1y_ini.size();

    // copies of x and g(y) with two levels of taping
    a2vector   a2y(n), a2z(n);

    // y, y^{(k)} , z^{(k)}, and y^{(k+1)}
    a1vector  a1y(n), a1y_k(n), a1z_k(n), a1y_kp(n);

    // initialize x
    for(i = 0; i < n; i++)
        a1y[i] = a1y_ini[i];

    // loop with respect to each step of Taylors method
    for(ell = 0; ell < nstep; ell++)
    {   // prepare to compute derivatives using a1double
        for(i = 0; i < n; i++)
            a2y[i] = a1y[i];
        CppAD::Independent(a2y);

        // evaluate ODE in a2double
        a2z = G(a2y);

        // define differentiable version of a1g: y -> z
        // that computes its derivatives using a1double objects
        CppAD::ADFun<a1double> a1g(a2y, a2z);

        // Use Taylor's method to take a step
        a1y_k            = a1y;     // initialize y^{(k)}
        a1double a1dt_kp = a1dt;  // initialize dt^(k+1)
        for(k = 0; k <= order; k++)
        {   // evaluate k-th order Taylor coefficient of y
            a1z_k = a1g.Forward(k, a1y_k);

            for(i = 0; i < n; i++)
            {   // convert to (k+1)-Taylor coefficient for x
                a1y_kp[i] = a1z_k[i] / a1double(k + 1);

                // add term for to this Taylor coefficient
                // to solution for y(t, x)
                a1y[i]    += a1y_kp[i] * a1dt_kp;
            }
            // next power of t
            a1dt_kp *= a1dt;
            // next Taylor coefficient
            a1y_k   = a1y_kp;
        }
    }
    return a1y;
}
} // END empty namespace
// ==========================================================================
// Routine that tests alogirhtmic differentiation of solutions computed
// by the routine taylor_ode.
bool mul_level_ode(void)
{   bool ok = true;
    double eps = 100. * std::numeric_limits<double>::epsilon();

    // number of components in differential equation
    size_t n = 4;

    // some temporary indices
    size_t i, j;

    // parameter vector in both double and a1double
    d_vector  x(n);
    a1vector  a1x(n);
    for(i = 0; i < n; i++)
        a1x[i] = x[i] = double(i + 1);

    // declare the parameters as the independent variable
    CppAD::Independent(a1x);

    // arguments to taylor_ode
    Ode G(a1x);                // function that defines the ODE
    size_t   order = n;      // order of Taylor's method used
    size_t   nstep = 2;      // number of steps to take
    a1double a1dt  = double(1.);     // Delta t for each step
    // value of y(t, x) at the initial time
    a1vector a1y_ini(n);
    for(i = 0; i < n; i++)
        a1y_ini[i] = 0.;

    // integrate the differential equation
    a1vector a1y_final(n);
    a1y_final = taylor_ode(G, order, nstep, a1dt, a1y_ini);

    // define differentiable fucntion object f : x -> y_final
    // that computes its derivatives in double
    CppAD::ADFun<double> f(a1x, a1y_final);

    // check function values
    double check = 1.;
    double t     = double(nstep) * Value(a1dt);
    for(i = 0; i < n; i++)
    {   check *= x[i] * t / double(i + 1);
        ok &= CppAD::NearEqual(Value(a1y_final[i]), check, eps, eps);
    }

    // evaluate the Jacobian of h at a
    d_vector jac ( f.Jacobian(x) );
    // There appears to be a bug in g++ version 4.4.2 because it generates
    // a warning for the equivalent form
    // d_vector jac = f.Jacobian(x);

    // check Jacobian
    for(i = 0; i < n; i++)
    {   for(j = 0; j < n; j++)
        {   double jac_ij = jac[i * n + j];
            if( i < j )
                check = 0.;
            else
                check = Value( a1y_final[i] ) / x[j];
            ok &= CppAD::NearEqual(jac_ij, check, eps, eps);
        }
    }
    return ok;
}

// END C++
