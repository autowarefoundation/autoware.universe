/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cstddef>                    // for size_t
# include <cmath>                      // for exp
# include <cppad/utility/ode_err_control.hpp>  // CppAD::OdeErrControl
# include <cppad/utility/near_equal.hpp>       // CppAD::NearEqual
# include <cppad/utility/vector.hpp>           // CppAD::vector
# include <cppad/utility/runge_45.hpp>         // CppAD::Runge45

/* -------------------------------------------------------------------------
Test relative error with zero initial conditions.
(Uses minimum step size to integrate).
*/

namespace {
    // --------------------------------------------------------------
    class Fun_one {
    private:
         size_t n;   // dimension of the state space
    public:
        // constructor
        Fun_one(size_t n_) : n(n_)
        { }

        // given x(0) = 0
        // solution is x_i (t) = t^(i+1)
        void Ode(
            const double                &t,
            const CppAD::vector<double> &x,
            CppAD::vector<double>       &f)
        {   size_t i;
            f[0] = 1.;
            for(i = 1; i < n; i++)
                f[i] = double(i+1) * x[i-1];
        }
    };

    // --------------------------------------------------------------
    class Method_one {
    private:
        Fun_one F;
    public:
        // constructor
        Method_one(size_t n_) : F(n_)
        { }
        void step(
            double ta,
            double tb,
            CppAD::vector<double> &xa ,
            CppAD::vector<double> &xb ,
            CppAD::vector<double> &eb )
        {   xb = CppAD::Runge45(F, 1, ta, tb, xa, eb);
        }
        size_t order(void)
        {   return 4; }
    };
}

bool OdeErrControl_one(void)
{   bool   ok = true;     // initial return value

    using CppAD::NearEqual;

    // Runge45 should yield exact results for x_i (t) = t^(i+1), i < 4
    size_t  n = 6;

    // construct method for n component solution
    Method_one method(n);

    // inputs to OdeErrControl

    double ti   = 0.;
    double tf   = .9;
    double smin = 1e-2;
    double smax = 1.;
    double scur = .5;
    double erel = 1e-7;

    CppAD::vector<double> xi(n);
    CppAD::vector<double> eabs(n);
    size_t i;
    for(i = 0; i < n; i++)
    {   xi[i]   = 0.;
        eabs[i] = 0.;
    }

    // outputs from OdeErrControl

    CppAD::vector<double> ef(n);
    CppAD::vector<double> xf(n);

    xf = OdeErrControl(method,
        ti, tf, xi, smin, smax, scur, eabs, erel, ef);

    double check = 1.;
    for(i = 0; i < n; i++)
    {   check *= tf;
        ok &= NearEqual(check, xf[i], erel, 0.);
    }

    return ok;
}

/*
Old example now just a test
Define
$latex X : \B{R} \rightarrow \B{R}^2$$ by
$latex \[
\begin{array}{rcl}
X_0 (t) & = &  - \exp ( - w_0 t )  \\
X_1 (t) & = & \frac{w_0}{w_1 - w_0} [ \exp ( - w_0 t ) - \exp( - w_1 t )]
\end{array}
\] $$
It follows that $latex X_0 (0) = 1$$, $latex X_1 (0) = 0$$ and
$latex \[
\begin{array}{rcl}
    X_0^{(1)} (t) & = & - w_0 X_0 (t)  \\
    X_1^{(1)} (t) & = & + w_0 X_0 (t) - w_1 X_1 (t)
\end{array}
\] $$
*/

namespace {
    // --------------------------------------------------------------
    class Fun_two {
    private:
         CppAD::vector<double> w;
    public:
        // constructor
        Fun_two(const CppAD::vector<double> &w_) : w(w_)
        { }

        // set f = x'(t)
        void Ode(
            const double                &t,
            const CppAD::vector<double> &x,
            CppAD::vector<double>       &f)
        {   f[0] = - w[0] * x[0];
            f[1] = + w[0] * x[0] - w[1] * x[1];
        }
    };

    // --------------------------------------------------------------
    class Method_two {
    private:
        Fun_two F;
    public:
        // constructor
        Method_two(const CppAD::vector<double> &w_) : F(w_)
        { }
        void step(
            double ta,
            double tb,
            CppAD::vector<double> &xa ,
            CppAD::vector<double> &xb ,
            CppAD::vector<double> &eb )
        {   xb = CppAD::Runge45(F, 1, ta, tb, xa, eb);
        }
        size_t order(void)
        {   return 4; }
    };
}

bool OdeErrControl_two(void)
{   bool ok = true;     // initial return value
    using CppAD::NearEqual;

    CppAD::vector<double> w(2);
    w[0] = 10.;
    w[1] = 1.;
    Method_two method(w);

    CppAD::vector<double> xi(2);
    xi[0] = 1.;
    xi[1] = 0.;

    CppAD::vector<double> eabs(2);
    eabs[0] = 1e-4;
    eabs[1] = 1e-4;

    // inputs
    double ti   = 0.;
    double tf   = 1.;
    double smin = 1e-4;
    double smax = 1.;
    double scur = .5;
    double erel = 0.;

    // outputs
    CppAD::vector<double> ef(2);
    CppAD::vector<double> xf(2);
    CppAD::vector<double> maxabs(2);
    size_t nstep;


    xf = OdeErrControl(method,
        ti, tf, xi, smin, smax, scur, eabs, erel, ef, maxabs, nstep);

    double x0 = exp(-w[0]*tf);
    ok &= NearEqual(x0, xf[0], 1e-4, 1e-4);
    ok &= NearEqual(0., ef[0], 1e-4, 1e-4);

    double x1 = w[0] * (exp(-w[0]*tf) - exp(-w[1]*tf))/(w[1] - w[0]);
    ok &= NearEqual(x1, xf[1], 1e-4, 1e-4);
    ok &= NearEqual(0., ef[1], 1e-4, 1e-4);

    return ok;
}
/*
Define
$latex X : \B{R} \rightarrow \B{R}^2$$ by
$latex \[
\begin{array}{rcl}
    X_0 (0)       & = & 1  \\
    X_1 (0)       & = & 0  \\
    X_0^{(1)} (t) & = & 2 \alpha t X_0 (t)  \\
    X_1^{(1)} (t) & = &  1 / X_0 (t)
\end{array}
\] $$
It follows that
$latex \[
\begin{array}{rcl}
X_0 (t) & = &  \exp ( \alpha t^2 )  \\
X_1 (t) & = &  \int_0^t \exp( - \alpha s^2 ) {\bf d} s \\
& = &
\frac{1}{ \sqrt{\alpha} \int_0^{\sqrt{\alpha} t} \exp( - r^2 ) {\bf d} r
\\
& = & \frac{\sqrt{\pi}}{ 2 \sqrt{\alpha} {\rm erf} ( \sqrt{\alpha} t )
\end{array}
\] $$
If $latex X_0 (t) < 0$$,
we return $code nan$$ in order to inform
$code OdeErrControl$$ that its is taking to large a step.

*/

# include <cppad/utility/rosen_34.hpp>          // CppAD::Rosen34
# include <cppad/cppad.hpp>

namespace {
    // --------------------------------------------------------------
    class Fun_three {
    private:
        const double alpha_;
        bool was_negative_;
    public:
        // constructor
        Fun_three(double alpha) : alpha_(alpha), was_negative_(false)
        { }

        // set f = x'(t)
        void Ode(
            const double                &t,
            const CppAD::vector<double> &x,
            CppAD::vector<double>       &f)
        {   f[0] = 2. * alpha_ * t * x[0];
            f[1] = 1. / x[0];
            // case where ODE does not make sense
            if( x[0] < 0. || x[1] < 0. )
            {   was_negative_ = true;
                f[0] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        // set f_t = df / dt
        void Ode_ind(
            const double                &t,
            const CppAD::vector<double> &x,
            CppAD::vector<double>       &f_t)
        {
            f_t[0] =  2. * alpha_ * x[0];
            f_t[1] = 0.;
            if( x[0] < 0. || x[1] < 0. )
            {   was_negative_ = true;
                f_t[0] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        // set f_x = df / dx
        void Ode_dep(
            const double                &t,
            const CppAD::vector<double> &x,
            CppAD::vector<double>       &f_x)
        {   double x0_sq = x[0] * x[0];
            f_x[0 * 2 + 0] = 2. * alpha_ * t;   // f0 w.r.t. x0
            f_x[0 * 2 + 1] = 0.;                // f0 w.r.t. x1
            f_x[1 * 2 + 0] = -1./x0_sq;         // f1 w.r.t. x0
            f_x[1 * 2 + 1] = 0.;                // f1 w.r.t. x1
            if( x[0] < 0. || x[1] < 0. )
            {   was_negative_ = true;
                f_x[0] = std::numeric_limits<double>::quiet_NaN();
            }
        }
        bool was_negative(void)
        {   return was_negative_; }

    };

    // --------------------------------------------------------------
    class Method_three {
    public:
        Fun_three F;

        // constructor
        Method_three(double alpha) : F(alpha)
        { }
        void step(
            double ta,
            double tb,
            CppAD::vector<double> &xa ,
            CppAD::vector<double> &xb ,
            CppAD::vector<double> &eb )
        {   xb = CppAD::Rosen34(F, 1, ta, tb, xa, eb);
        }
        size_t order(void)
        {   return 3; }
    };
}

bool OdeErrControl_three(void)
{   bool ok = true;     // initial return value
    using CppAD::NearEqual;

    double alpha = 10.;
    Method_three method(alpha);

    CppAD::vector<double> xi(2);
    xi[0] = 1.;
    xi[1] = 0.;

    CppAD::vector<double> eabs(2);
    eabs[0] = 1e-4;
    eabs[1] = 1e-4;

    // inputs
    double ti   = 0.;
    double tf   = 1.;
    double smin = 1e-4;
    double smax = 1.;
    double scur = 1.;
    double erel = 0.;

    // outputs
    CppAD::vector<double> ef(2);
    CppAD::vector<double> xf(2);
    CppAD::vector<double> maxabs(2);
    size_t nstep;


    xf = OdeErrControl(method,
        ti, tf, xi, smin, smax, scur, eabs, erel, ef, maxabs, nstep);


    double x0       = exp( alpha * tf * tf );
    ok &= NearEqual(x0, xf[0], 1e-4, 1e-4);
    ok &= NearEqual(0., ef[0], 1e-4, 1e-4);

    double root_pi    = sqrt( 4. * atan(1.));
    double root_alpha = sqrt( alpha );
    double x1 = CppAD::erf(alpha * tf) * root_pi / (2 * root_alpha);
    ok &= NearEqual(x1, xf[1], 1e-4, 1e-4);
    ok &= NearEqual(0., ef[1], 1e-4, 1e-4);

    ok &= method.F.was_negative();

    return ok;
}

namespace {
    // --------------------------------------------------------------
    class Fun_four {
    private:
         size_t n;   // dimension of the state space
    public:
        // constructor
        Fun_four(size_t n_) : n(n_)
        { }

        // given x(0) = 0
        // solution is x_i (t) = t^(i+1)
        void Ode(
            const double                &t,
            const CppAD::vector<double> &x,
            CppAD::vector<double>       &f)
        {   size_t i;
            f[0] = std::numeric_limits<double>::quiet_NaN();
            for(i = 1; i < n; i++)
                f[i] = double(i+1) * x[i-1];
        }
    };

    // --------------------------------------------------------------
    class Method_four {
    private:
        Fun_four F;
    public:
        // constructor
        Method_four(size_t n_) : F(n_)
        { }
        void step(
            double ta,
            double tb,
            CppAD::vector<double> &xa ,
            CppAD::vector<double> &xb ,
            CppAD::vector<double> &eb )
        {   xb = CppAD::Runge45(F, 1, ta, tb, xa, eb);
        }
        size_t order(void)
        {   return 4; }
    };
}

bool OdeErrControl_four(void)
{   bool   ok = true;     // initial return value

    // construct method for n component solution
    size_t  n = 6;
    Method_four method(n);

    // inputs to OdeErrControl

    // special case where scur is converted to ti - tf
    // (so it is not equal to smin)
    double ti   = 0.;
    double tf   = .9;
    double smin = .8;
    double smax = 1.;
    double scur = smin;
    double erel = 1e-7;

    CppAD::vector<double> xi(n);
    CppAD::vector<double> eabs(n);
    size_t i;
    for(i = 0; i < n; i++)
    {   xi[i]   = 0.;
        eabs[i] = 0.;
    }

    // outputs from OdeErrControl
    CppAD::vector<double> ef(n);
    CppAD::vector<double> xf(n);

    xf = OdeErrControl(method,
        ti, tf, xi, smin, smax, scur, eabs, erel, ef);

    // check that Fun_four always returning nan results in nan
    for(i = 0; i < n; i++)
    {   ok &= CppAD::isnan(xf[i]);
        ok &= CppAD::isnan(ef[i]);
    }

    return ok;
}

// ==========================================================================
bool ode_err_control(void)
{   bool ok = true;
    ok     &= OdeErrControl_one();
    ok     &= OdeErrControl_two();
    ok     &= OdeErrControl_three();
    ok     &= OdeErrControl_four();
    return ok;
}
