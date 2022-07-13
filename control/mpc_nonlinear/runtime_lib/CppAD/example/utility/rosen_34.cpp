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
$begin rosen_34.cpp$$
$spell
    Rosen
$$

$section rosen_34: Example and Test$$


Define
$latex X : \B{R} \rightarrow \B{R}^n$$ by
$latex \[
    X_i (t) =  t^{i+1}
\] $$
for $latex i = 1 , \ldots , n-1$$.
It follows that
$latex \[
\begin{array}{rclr}
X_i(0)     & = & 0                             & {\rm for \; all \;} i \\
X_i ' (t)  & = & 1                             & {\rm if \;} i = 0      \\
X_i '(t)   & = & (i+1) t^i = (i+1) X_{i-1} (t) & {\rm if \;} i > 0
\end{array}
\] $$
The example tests Rosen34 using the relations above:

$head Operation Sequence$$
The $cref rosen34$$ method for solving ODE's requires the inversion
of a system of linear equations.
This indices used for pivoting may change with different values
for $latex t$$ and $latex x$$.
This example checks the comparison operators.
If some of the comparisons change,
it makes a new recording of the function with the pivots for the current
$latex t$$ and $latex x$$.
Note that one could skip this step and always use the same pivot.
This would not be as numerically stable,
but it would still solve the equations
(so long as none of the pivot elements are zero).

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>        // For automatic differentiation

namespace {
    class Fun {
    private:
        const bool           use_x_;
        CppAD::ADFun<double> ode_ind_;
        CppAD::ADFun<double> ode_dep_;
    public:
        // constructor
        Fun(bool use_x) : use_x_(use_x)
        { }

        // compute f(t, x) both for double and AD<double>
        template <class Scalar>
        void Ode(
            const Scalar                   &t,
            const CPPAD_TESTVECTOR(Scalar) &x,
            CPPAD_TESTVECTOR(Scalar)       &f)
        {   size_t n  = x.size();
            Scalar ti(1);
            f[0]   = Scalar(1);
            for(size_t i = 1; i < n; i++)
            {   ti *= t;
                if( use_x_ )
                    f[i] = Scalar(i+1) * x[i-1];
                else
                    f[i] = Scalar(i+1) * ti;
            }
        }

        // compute partial of f(t, x) w.r.t. t using AD
        void Ode_ind(
            const double                   &t,
            const CPPAD_TESTVECTOR(double) &x,
            CPPAD_TESTVECTOR(double)       &f_t)
        {   using namespace CppAD;

            size_t n                = x.size();
            bool   ode_ind_defined  = ode_ind_.size_var() != 0;
            //
            CPPAD_TESTVECTOR(double) t_vec(1);
            t_vec[0] = t;
            //
            bool retape = true;
            if( ode_ind_defined )
            {   // check if any comparison operators have a different result
                ode_ind_.new_dynamic(x);
                ode_ind_.Forward(0, t_vec);
                retape = ode_ind_.compare_change_number() > 0;
            }
            if( retape )
            {   // record function that evaluates f(t, x)
                // with t as independent variable and x as dynamcic parameter
                CPPAD_TESTVECTOR(AD<double>) at(1);
                CPPAD_TESTVECTOR(AD<double>) ax(n);
                CPPAD_TESTVECTOR(AD<double>) af(n);

                // set argument values
                at[0] = t;
                size_t i;
                for(i = 0; i < n; i++)
                    ax[i] = x[i];

                // declare independent variables and dynamic parameters
                size_t abort_op_index = 0;
                bool   record_compare = false;
                Independent(at, abort_op_index, record_compare, ax);

                // compute f(t, x)
                this->Ode(at[0], ax, af);

                // define AD function object
                ode_ind_.Dependent(at, af);

                // store result in ode_ind_ so can be re-used
                assert( ode_ind_.size_var() != 0 );
            }
            // special case where new_dynamic not yet set
            if( ! ode_ind_defined )
                ode_ind_.new_dynamic(x);
            // compute partial of f w.r.t t
            f_t = ode_ind_.Jacobian(t_vec); // partial f(t, x) w.r.t. t
        }

        // compute partial of f(t, x) w.r.t. x using AD
        void Ode_dep(
            const double                   &t,
            const CPPAD_TESTVECTOR(double) &x,
            CPPAD_TESTVECTOR(double)       &f_x)
        {   using namespace CppAD;

            size_t n                = x.size();
            bool   ode_dep_defined  = ode_dep_.size_var() != 0;
            //
            CPPAD_TESTVECTOR(double) t_vec(1), dx(n), df(n);
            t_vec[0] = t;
            //
            bool retape = true;
            if( ode_dep_defined )
            {   // check if any comparison operators have a differrent result
                ode_dep_.new_dynamic(t_vec);
                ode_dep_.Forward(0, x);
                retape = ode_dep_.compare_change_number() > 0;
            }
            if( retape )
            {   // record function that evaluates f(t, x)
                // with x as independent variable and t as dynamcic parameter
                CPPAD_TESTVECTOR(AD<double>) at(1);
                CPPAD_TESTVECTOR(AD<double>) ax(n);
                CPPAD_TESTVECTOR(AD<double>) af(n);

                // set argument values
                at[0] = t;
                for(size_t i = 0; i < n; i++)
                    ax[i] = x[i];

                // declare independent variables
                size_t abort_op_index = 0;
                bool   record_compare = false;
                Independent(ax, abort_op_index, record_compare, at);

                // compute f(t, x)
                this->Ode(at[0], ax, af);

                // define AD function object
                ode_dep_.Dependent(ax, af);

                // store result in ode_dep_ so can be re-used
                assert( ode_ind_.size_var() != 0 );
            }
            // special case where new_dynamic not yet set
            if( ! ode_dep_defined )
                ode_dep_.new_dynamic(t_vec);
            // compute partial of f w.r.t x
            f_x = ode_dep_.Jacobian(x); // partial f(t, x) w.r.t. x
        }


    };
}

bool rosen_34(void)
{   bool ok = true;     // initial return value

    using CppAD::NearEqual;
    double eps99 = 99.0 * std::numeric_limits<double>::epsilon();

    size_t  n = 4;      // number components in X(t) and order of method
    size_t  M = 2;      // number of Rosen34 steps in [ti, tf]
    double ti = 0.;     // initial time
    double tf = 2.;     // final time

    // xi = X(0)
    CPPAD_TESTVECTOR(double) xi(n);
    for(size_t i = 0; i <n; i++)
        xi[i] = 0.;

    for(size_t use_x = 0; use_x < 2; use_x++)
    {   // function object depends on value of use_x
        Fun F(use_x > 0);

        // compute Rosen34 approximation for X(tf)
        CPPAD_TESTVECTOR(double) xf(n), e(n);
        xf = CppAD::Rosen34(F, M, ti, tf, xi, e);

        double check = tf;
        for(size_t i = 0; i < n; i++)
        {   // check that error is always positive
            ok    &= (e[i] >= 0.);
            // 4th order method is exact for i < 4
            if( i < 4 ) ok &=
                NearEqual(xf[i], check, eps99, eps99);
            // 3rd order method is exact for i < 3
            if( i < 3 )
                ok &= (e[i] <= eps99);

            // check value for next i
            check *= tf;
        }
    }
    return ok;
}

// END C++
