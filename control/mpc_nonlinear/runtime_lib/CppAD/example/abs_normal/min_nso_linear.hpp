# ifndef CPPAD_EXAMPLE_ABS_NORMAL_MIN_NSO_LINEAR_HPP
# define CPPAD_EXAMPLE_ABS_NORMAL_MIN_NSO_LINEAR_HPP
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
$begin min_nso_linear$$
$spell
    hpp
    nso
    jac
    Jacobian
    maxitr
    smo
$$
$section Non-Smooth Optimization Using Abs-normal Linear Approximations$$

$head Syntax$$
$icode%ok% = min_nso_linear(
    %level%, %g%, %a%, %epsilon%, %maxitr%, %b_in%, %x_in%, %x_out%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%
1%$$

$head Source$$
This following is a link to the source code for this example:
$cref/min_nso_linear.hpp/min_nso_linear.hpp/$$.

$head Purpose$$
Given a current that abs-normal representation
$cref/g/abs_normal_fun/g/$$, $cref/a/abs_normal_fun/a/$$,
for a function $latex f(x)$$,
this routine minimizes $latex f(x)$$.

$head DblVector$$
is a $cref SimpleVector$$ class with elements of type $code double$$.

$head SizeVector$$
is a $cref SimpleVector$$ class with elements of type $code size_t$$.

$head f$$
We use the notation $icode f$$ for the original function; see
$cref/f/abs_normal_fun/f/$$.

$subhead n$$
We use $icode n$$ to denote the dimension of the domain for $icode f$$; i.e.,
$icode%f%.Domain()%$$.

$subhead m$$
We use $icode m$$ to denote the dimension of the range for $icode f$$; i.e.,
$icode%f%.Range()%$$.
This must be equal to one.

$subhead s$$
We use
$cref/s/abs_normal_fun/f/s/$$ to denote the number absolute terms in $icode f$$.

$head level$$
This value is less that or equal 5.
If $icode%level% == 0%$$,
no tracing of the optimization is printed.
If $icode%level% >= 1%$$,
a trace of each iteration of $code min_nso_linear$$ is printed.
If $icode%level% >= 2%$$,
a trace of each iteration of the $code abs_min_linear$$ sub-problem is printed.
If $icode%level% >= 3%$$,
a trace of the $cref lp_box$$ sub-problem is printed.
If $icode%level% >= 4%$$,
a trace of the objective and primal variables $latex x$$ are printed
at each $cref simplex_method$$ iteration.
If $icode%level% == 5%$$,
the simplex tableau is printed at each simplex iteration.

$head g$$
This is the function $cref/g/abs_normal_fun/g/$$
in the abs-normal representation of $icode f$$.

$head a$$
This is the function $cref/a/abs_normal_fun/a/$$
in the abs-normal representation of $icode f$$.

$head epsilon$$
This is a vector with size 2.
The value $icode%epsilon%[0]%$$ is convergence criteria in terms
of the infinity norm of the difference of $icode x_out$$
between iterations.
The value $icode%epsilon%[1]%$$ is convergence criteria in terms
of the derivative of $latex f(x)$$.
This derivative is actually the average of the directional derivative
in the direction of the sub-problem minimizer.

$head maxitr$$
This is a vector with size 3.
The value $icode%maxitr%[0]%$$ is the maximum number of
$code min_nso_linear$$ iterations to try before giving up on convergence.
The value $icode%maxitr%[1]%$$ is the maximum number of iterations in the
$code abs_min_linear$$ sub-problem.
The value $icode%maxitr%[2]%$$ is the maximum number of iterations in
the $cref/simplex_method/simplex_method/maxitr/$$ sub-problems.

$head b_in$$
This the initial bound on the trust region size.
To be specific, if $latex b$$ is the current trust region size,
at each iteration affine approximation is minimized with respect to
$latex \Delta x$$ and subject to
$latex \[
    -b \leq \Delta x_j \leq b
\] $$
for $icode%j% = 0 , %...%, %n%-1%$$.
It must hold that $icode%b_in% > %epsilon%[0]%$$.

$head x_in$$
This vector $icode x_out$$ has size $icode n$$.
It is the starting point for the optimization procedure; i.e.,
the $code min_nso_linear$$ iterations.

$head x_out$$
This vector $icode x_out$$ has size $icode n$$.
The input value of its elements does not matter.
Upon return,
it is the approximate minimizer
of the abs-normal approximation for $latex f(x)$$ over the trust region
is $latex x = \hat{x} + \Delta x$$.

$children%example/abs_normal/min_nso_linear.cpp
    %example/abs_normal/min_nso_linear.omh
%$$
$head Example$$
The file $cref min_nso_linear.cpp$$ contains an example and test of
$code min_nso_linear$$.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/cppad.hpp>
# include "abs_min_linear.hpp"
# include "abs_eval.hpp"

// BEGIN C++
namespace {
    CPPAD_TESTVECTOR(double) min_nso_linear_join(
        const CPPAD_TESTVECTOR(double)& x ,
        const CPPAD_TESTVECTOR(double)& u )
    {   size_t n = x.size();
        size_t s = u.size();
        CPPAD_TESTVECTOR(double) xu(n + s);
        for(size_t j = 0; j < n; j++)
            xu[j] = x[j];
        for(size_t j = 0; j < s; j++)
            xu[n + j] = u[j];
        return xu;
    }
}

// BEGIN C++
namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// BEGIN PROTOTYPE
template <class DblVector, class SizeVector>
bool min_nso_linear(
    size_t           level     ,
    ADFun<double>&   g         ,
    ADFun<double>&   a         ,
    const DblVector& epsilon   ,
    SizeVector       maxitr    ,
    double           b_in      ,
    const DblVector& x_in      ,
    DblVector&       x_out     )
// END PROTOTYPE
{
    using std::fabs;
    //
    // number of absolute value terms
    size_t s  = a.Range();
    //
    // size of domain for f
    size_t n  = g.Domain() - s;
    //
    // size of range space for f
    size_t m = g.Range() - s;
    //
    CPPAD_ASSERT_KNOWN(
        level <= 5,
        "min_nso_linear: level is not less that or equal 5"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(epsilon.size()) == 2,
        "min_nso_linear: size of epsilon not equal to 2"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(maxitr.size()) == 3,
        "min_nso_linear: size of maxitr not equal to 3"
    );
    CPPAD_ASSERT_KNOWN(
        g.Domain() > s && g.Range() > s,
        "min_nso_linear: g, a is not an abs-normal representation"
    );
    CPPAD_ASSERT_KNOWN(
        m == 1,
        "min_nso_linear: m is not equal to 1"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(x_in.size()) == n,
        "min_nso_linear: size of x_in not equal to n"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(x_out.size()) == n,
        "min_nso_linear: size of x_out not equal to n"
    );
    CPPAD_ASSERT_KNOWN(
        epsilon[0] < b_in,
        "min_nso_linear: b_in <= epsilon[0]"
    );
    if( level > 0 )
    {   std::cout << "start min_nso_linear\n";
        std::cout << "b_in = " << b_in << "\n";
        CppAD::abs_print_mat("x_in", n, 1, x_in);
    }
    // level in abs_min_linear sub-problem
    size_t level_tilde = 0;
    if( level > 0 )
        level_tilde = level - 1;
    //
    // maxitr in abs_min_linear sub-problem
    SizeVector maxitr_tilde(2);
    maxitr_tilde[0] = maxitr[1];
    maxitr_tilde[1] = maxitr[2];
    //
    // epsilon in abs_min_linear sub-problem
    DblVector eps_tilde(2);
    eps_tilde[0] = epsilon[0] / 10.;
    eps_tilde[1] = epsilon[1] / 10.;
    //
    // current bound
    double b_cur = b_in;
    //
    // initilaize the current x
    x_out = x_in;
    //
    // value of a(x) at current x
    DblVector a_cur = a.Forward(0, x_out);
    //
    // (x_out, a_cur)
    DblVector xu_cur = min_nso_linear_join(x_out, a_cur);
    //
    // value of g[ x_cur, a_cur ]
    DblVector g_cur = g.Forward(0, xu_cur);
    //
    for(size_t itr = 0; itr < maxitr[0]; itr++)
    {
        // Jacobian of g[ x_cur, a_cur ]
        DblVector g_jac = g.Jacobian(xu_cur);
        //
        // bound in abs_min_linear sub-problem
        DblVector bound_tilde(n);
        for(size_t j = 0; j < n; j++)
            bound_tilde[j] = b_cur;
        //
        DblVector delta_x(n);
        bool ok = abs_min_linear(
            level_tilde, n, m, s,
            g_cur, g_jac, bound_tilde, eps_tilde, maxitr_tilde, delta_x
        );
        if( ! ok )
        {   if( level > 0 )
                std::cout << "end min_nso_linear: abs_min_linear failed\n";
            return false;
        }
        //
        // new candidate value for x
        DblVector x_new(n);
        double max_delta_x = 0.0;
        for(size_t j = 0; j < n; j++)
        {   x_new[j] = x_out[j] + delta_x[j];
            max_delta_x = std::max(max_delta_x, std::fabs( delta_x[j] ) );
        }
        //
        if( max_delta_x < b_cur && max_delta_x < epsilon[0] )
        {   if( level > 0 )
                std::cout << "end min_nso_linear: delta_x is near zero\n";
            return true;
        }
        // value of abs-normal approximation at minimizer
        DblVector g_tilde = CppAD::abs_eval(n, m, s, g_cur, g_jac, delta_x);
        //
        double derivative = (g_tilde[0] - g_cur[0]) / max_delta_x;
        CPPAD_ASSERT_UNKNOWN( derivative <= 0.0 )
        if( - epsilon[1] < derivative )
        {   if( level > 0 )
                std::cout << "end min_nso_linear: derivative near zero\n";
            return true;
        }
        //
        // value of a(x) at new x
        DblVector a_new = a.Forward(0, x_new);
        //
        // (x_new, a_new)
        DblVector xu_new = min_nso_linear_join(x_new, a_new);
        //
        // value of g[ x_new, a_new ]
        DblVector g_new = g.Forward(0, xu_new);
        //
        //
        // check for descent of objective
        double rate_new = (g_new[0] - g_cur[0]) / max_delta_x;
        if( - epsilon[1] < rate_new )
        {   // did not get sufficient descent
            b_cur /= 2.0;
            if( level > 0 )
                std::cout << "itr = " << itr
                << ", rate_new = " << rate_new
                << ", b_cur = " << b_cur << "\n";
            //
        }
        else
        {   // got sufficient descent so accept candidate for x
            x_out  = x_new;
            a_cur  = a_new;
            g_cur  = g_new;
            xu_cur = xu_new;
            //
            if( level >  0 )
            {   std::cout << "itr = " << itr
                << ", derivative = "<< derivative
                << ", max_delta_x = "<< max_delta_x
                << ", objective = " << g_cur[0] << "\n";
                abs_print_mat("x_out", n, 1, x_out);
            }
        }
    }
    if( level > 0 )
        std::cout << "end min_nso_linear: maximum number of iterations exceeded\n";
    return false;
}
} // END_CPPAD_NAMESPACE
// END C++

# endif
