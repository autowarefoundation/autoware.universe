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
$begin base2ad.cpp$$
$spell
    Taylor
    Cpp
    const
    std
    Adolc
    adouble
$$

$section Taylor's Ode Solver: base2ad Example and Test$$

$head See Also$$
$cref taylor_ode.cpp$$, $cref mul_level_ode.cpp$$

$head Purpose$$
This is a realistic example using $cref base2ad$$ to create
an $codei%AD<%Base%>%$$ function from an $icode Base$$ function.
The function represents an ordinary differential equation.
It is differentiated with respect to
its $cref/variables/glossary/Variable/$$.
These derivatives are used by the $cref taylor_ode$$ method.
This solution is then differentiated with respect to
the functions $cref/dynamic parameters/glossary/Parameter/Dynamic/$$.

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
    z ( t , x ) = g[ y ( t , x ), x ]
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
namespace { // BEGIN empty namespace

typedef CppAD::AD<double>                  a_double;

typedef CPPAD_TESTVECTOR(double)           d_vector;
typedef CPPAD_TESTVECTOR(a_double)         a_vector;

typedef CppAD::ADFun<double>               fun_double;
typedef CppAD::ADFun<a_double, double>     afun_double;

// -------------------------------------------------------------------------
// class definition for C++ function object that defines ODE
class Ode {
private:
    // copy of x that is set by constructor and used by g(y)
    a_vector x_;
public:
    // constructor
    Ode(const a_vector& x) : x_(x)
    { }
    // the function g(y) given the parameter vector x
    a_vector operator() (const a_vector& y) const
    {   size_t n = y.size();
        a_vector g(n);
        g[0] = x_[0];
        for(size_t i = 1; i < n; i++)
            g[i] = x_[i] * y[i-1];
        //
        return g;
    }
};

// -------------------------------------------------------------------------
// Routine that uses Taylor's method to solve ordinary differential equaitons
a_vector taylor_ode(
    afun_double&     fun_g   ,  // function that defines the ODE
    size_t           order   ,  // order of Taylor's method used
    size_t           nstep   ,  // number of steps to take
    const a_double&  dt      ,  // Delta t for each step
    const a_vector&  y_ini)     // y(t) at the initial time
{
    // number of variables in the ODE
    size_t n = y_ini.size();

    // initialize y
    a_vector y = y_ini;

    // loop with respect to each step of Taylors method
    for(size_t s = 0; s < nstep; s++)
    {
        // initialize
        a_vector y_k   = y;
        a_double dt_k  = a_double(1.0);
        a_vector next  = y;

        for(size_t k = 0; k < order; k++)
        {
            // evaluate k-th order Taylor coefficient z^{(k)} (t)
            a_vector z_k = fun_g.Forward(k, y_k);

            // dt^{k+1}
            dt_k *= dt;

            // y^{(k+1)}
            for(size_t i = 0; i < n; i++)
            {   // y^{(k+1)}
                y_k[i] = z_k[i] / a_double(k + 1);

                // add term for k+1 Taylor coefficient
                // to solution for next y
                next[i] += y_k[i] * dt_k;
            }
        }

        // take step
        y = next;
    }
    return y;
}
} // END empty namespace

// ==========================================================================
// Routine that tests alogirhtmic differentiation of solutions computed
// by the routine taylor_ode.
bool base2ad(void)
{   bool ok = true;
    double eps = 100. * std::numeric_limits<double>::epsilon();

    // number of components in differential equation
    size_t n = 4;

    // record function g(y, x)
    // with y as the independent variables and x as dynamic parameters
    a_vector  ay(n), ax(n);
    for(size_t i = 0; i < n; i++)
        ay[i] = ax[i] = double(i + 1);
    CppAD::Independent(ay, ax);

    // fun_g
    Ode G(ax);
    a_vector ag = G(ay);
    fun_double fun_g(ay, ag);


    // afun_g
    afun_double afun_g( fun_g.base2ad() ); // differential equation

    // other arguments to taylor_ode
    size_t   order = n;       // order of Taylor's method used
    size_t   nstep = 2;       // number of steps to take
    a_double adt   = 1.;      // Delta t for each step
    a_vector ay_ini(n);       // initial value of y
    for(size_t i = 0; i < n; i++)
        ay_ini[i] = 0.;

    // declare x as independent variables
    CppAD::Independent(ax);

    // the independent variables if this function are
    // the dynamic parameters in afun_g
    afun_g.new_dynamic(ax);

    // integrate the differential equation
    a_vector ay_final;
    ay_final = taylor_ode(afun_g, order, nstep, adt, ay_ini);

    // define differentiable fucntion object f(x) = y_final(x)
    // that computes its derivatives in double
    CppAD::ADFun<double> fun_f(ax, ay_final);

    // double version of ax
    d_vector x(n);
    for(size_t i = 0; i < n; i++)
        x[i] = Value( ax[i] );

    // check function values
    double check = 1.;
    double t     = double(nstep) * Value(adt);
    for(size_t i = 0; i < n; i++)
    {   check *= x[i] * t / double(i + 1);
        ok &= CppAD::NearEqual(Value(ay_final[i]), check, eps, eps);
    }

    // There appears to be a bug in g++ version 4.4.2 because it generates
    // a warning for the equivalent form
    // d_vector jac = fun_f.Jacobian(x);
    d_vector jac ( fun_f.Jacobian(x) );

    // check Jacobian
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n; j++)
        {   double jac_ij = jac[i * n + j];
            if( i < j )
                check = 0.;
            else
                check = Value( ay_final[i] ) / x[j];
            ok &= CppAD::NearEqual(jac_ij, check, eps, eps);
        }
    }
    return ok;
}

// END C++
