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
$begin ode_stiff.cpp$$
$spell
    Rosen
$$

$section A Stiff Ode: Example and Test$$


Define
$latex x : \B{R} \rightarrow \B{R}^2$$ by
$latex \[
\begin{array}{rcl}
    x_0 (0)        & = & 1 \\
    x_1 (0)        & = & 0 \\
    x_0^\prime (t) & = & - a_0 x_0 (t) \\
    x_1^\prime (t) & = & + a_0 x_0 (t) - a_1 x_1 (t)
\end{array}
\] $$
If $latex a_0 \gg a_1 > 0$$, this is a stiff Ode and
the analytic solution is
$latex \[
\begin{array}{rcl}
x_0 (t)    & = & \exp( - a_0 t ) \\
x_1 (t)    & = & a_0 [ \exp( - a_1 t ) - \exp( - a_0 t ) ] / ( a_0 - a_1 )
\end{array}
\] $$
The example tests Rosen34 using the relations above:

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

// To print the comparison, change the 0 to 1 on the next line.
# define CPPAD_ODE_STIFF_PRINT 0

namespace {
    // --------------------------------------------------------------
    class Fun {
    private:
        CPPAD_TESTVECTOR(double) a;
    public:
        // constructor
        Fun(const CPPAD_TESTVECTOR(double)& a_) : a(a_)
        { }
        // compute f(t, x)
        void Ode(
            const double                    &t,
            const CPPAD_TESTVECTOR(double) &x,
            CPPAD_TESTVECTOR(double)       &f)
        {   f[0]  = - a[0] * x[0];
            f[1]  = + a[0] * x[0] - a[1] * x[1];
        }
        // compute partial of f(t, x) w.r.t. t
        void Ode_ind(
            const double                    &t,
            const CPPAD_TESTVECTOR(double) &x,
            CPPAD_TESTVECTOR(double)       &f_t)
        {   f_t[0] = 0.;
            f_t[1] = 0.;
        }
        // compute partial of f(t, x) w.r.t. x
        void Ode_dep(
            const double                    &t,
            const CPPAD_TESTVECTOR(double) &x,
            CPPAD_TESTVECTOR(double)       &f_x)
        {   f_x[0] = -a[0];
            f_x[1] = 0.;
            f_x[2] = +a[0];
            f_x[3] = -a[1];
        }
    };
    // --------------------------------------------------------------
    class RungeMethod {
    private:
        Fun F;
    public:
        // constructor
        RungeMethod(const CPPAD_TESTVECTOR(double) &a_) : F(a_)
        { }
        void step(
            double                     ta ,
            double                     tb ,
            CPPAD_TESTVECTOR(double) &xa ,
            CPPAD_TESTVECTOR(double) &xb ,
            CPPAD_TESTVECTOR(double) &eb )
        {   xb = CppAD::Runge45(F, 1, ta, tb, xa, eb);
        }
        size_t order(void)
        {   return 5; }
    };
    class RosenMethod {
    private:
        Fun F;
    public:
        // constructor
        RosenMethod(const CPPAD_TESTVECTOR(double) &a_) : F(a_)
        { }
        void step(
            double                     ta ,
            double                     tb ,
            CPPAD_TESTVECTOR(double) &xa ,
            CPPAD_TESTVECTOR(double) &xb ,
            CPPAD_TESTVECTOR(double) &eb )
        {   xb = CppAD::Rosen34(F, 1, ta, tb, xa, eb);
        }
        size_t order(void)
        {   return 4; }
    };
}

bool OdeStiff(void)
{   bool ok = true;     // initial return value

    CPPAD_TESTVECTOR(double) a(2);
    a[0] = 1e3;
    a[1] = 1.;
    RosenMethod rosen(a);
    RungeMethod runge(a);
    Fun          gear(a);

    CPPAD_TESTVECTOR(double) xi(2);
    xi[0] = 1.;
    xi[1] = 0.;

    CPPAD_TESTVECTOR(double) eabs(2);
    eabs[0] = 1e-6;
    eabs[1] = 1e-6;

    CPPAD_TESTVECTOR(double) ef(2);
    CPPAD_TESTVECTOR(double) xf(2);
    CPPAD_TESTVECTOR(double) maxabs(2);
    size_t                nstep;

    size_t k;
    for(k = 0; k < 3; k++)
    {
        size_t M    = 5;
        double ti   = 0.;
        double tf   = 1.;
        double smin = 1e-7;
        double sini = 1e-7;
        double smax = 1.;
        double scur = .5;
        double erel = 0.;

        if( k == 0 )
        {   xf = CppAD::OdeErrControl(rosen, ti, tf,
            xi, smin, smax, scur, eabs, erel, ef, maxabs, nstep);
        }
        else if( k == 1 )
        {   xf = CppAD::OdeErrControl(runge, ti, tf,
            xi, smin, smax, scur, eabs, erel, ef, maxabs, nstep);
        }
        else if( k == 2 )
        {   xf = CppAD::OdeGearControl(gear, M, ti, tf,
            xi, smin, smax, sini, eabs, erel, ef, maxabs, nstep);
        }
        double x0 = exp(-a[0]*tf);
        ok &= CppAD::NearEqual(x0, xf[0], 0., eabs[0]);
        ok &= CppAD::NearEqual(0., ef[0], 0., eabs[0]);

        double x1 = a[0] *
            (exp(-a[1]*tf) - exp(-a[0]*tf))/(a[0] - a[1]);
        ok &= CppAD::NearEqual(x1, xf[1], 0., eabs[1]);
        ok &= CppAD::NearEqual(0., ef[1], 0., eabs[0]);
# if CPPAD_ODE_STIFF_PRINT
        const char* method[]={ "Rosen34", "Runge45", "Gear5" };
        std::cout << std::endl;
        std::cout << "method     = " << method[k] << std::endl;
        std::cout << "nstep      = " << nstep  << std::endl;
        std::cout << "x0         = " << x0 << std::endl;
        std::cout << "xf[0]      = " << xf[0] << std::endl;
        std::cout << "x0 - xf[0] = " << x0 - xf[0] << std::endl;
        std::cout << "ef[0]      = " << ef[0] << std::endl;
        std::cout << "x1         = " << x1 << std::endl;
        std::cout << "xf[1]      = " << xf[1] << std::endl;
        std::cout << "x1 - xf[1] = " << x1 - xf[1] << std::endl;
        std::cout << "ef[1]      = " << ef[1] << std::endl;
# endif
    }

    return ok;
}

// END C++
