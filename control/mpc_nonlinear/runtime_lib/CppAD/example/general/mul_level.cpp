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
$begin mul_level.cpp$$
$spell
    Adolc
    adouble
    CppAD
    abs
$$

$section Multiple Level of AD: Example and Test$$

$head See Also$$
$cref base2ad.cpp$$

$head Purpose$$
In this example, we use $code AD< AD<double> >$$ (level two taping),
the compute values of the function $latex f : \B{R}^n \rightarrow \B{R}$$ where
$latex \[
    f(x) = \frac{1}{2} \left( x_0^2 + \cdots + x_{n-1}^2 \right)
\] $$
We then use $code AD<double>$$ (level one taping) to compute
the directional derivative
$latex \[
f^{(1)} (x) * v  = x_0 v_0 + \cdots + x_{n-1} v_{n-1}
\] $$.
where $latex v \in \B{R}^n$$.
We then use $code double$$ (no taping) to compute
$latex \[
\frac{d}{dx} \left[ f^{(1)} (x) * v \right] = v
\] $$
This is only meant as an example of multiple levels of taping.
The example $cref hes_times_dir.cpp$$ computes the same value more
efficiently by using the identity:
$latex \[
    \frac{d}{dx} \left[ f^{(1)} (x) * v \right] = f^{(2)} (x) * v
\] $$
The example $cref mul_level_adolc.cpp$$ computes the same values using
Adolc's type $code adouble$$ and CppAD's type $code AD<adouble>$$.


$head Source$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>

namespace {
    // f(x) = |x|^2 / 2 = .5 * ( x[0]^2 + ... + x[n-1]^2 )
    template <class Type>
    Type f(const CPPAD_TESTVECTOR(Type)& x)
    {   Type sum;

        sum  = 0.;
        size_t i = size_t(x.size());
        for(i = 0; i < size_t(x.size()); i++)
            sum += x[i] * x[i];

        return .5 * sum;
    }
}

bool mul_level(void)
{   bool ok = true;                          // initialize test result

    typedef CppAD::AD<double>   a1type;    // for one level of taping
    typedef CppAD::AD<a1type>    a2type;    // for two levels of taping
    size_t n = 5;                           // dimension for example
    size_t j;                               // a temporary index variable

    // 10 times machine epsilon
    double eps = 10. * std::numeric_limits<double>::epsilon();

    CPPAD_TESTVECTOR(double) x(n);
    CPPAD_TESTVECTOR(a1type)  a1x(n), a1v(n), a1dy(1) ;
    CPPAD_TESTVECTOR(a2type)  a2x(n), a2y(1);

    // Values for the independent variables while taping the function f(x)
    for(j = 0; j < n; j++)
        a2x[j] = a1x[j] = x[j] = double(j);
    // Declare the independent variable for taping f(x)
    CppAD::Independent(a2x);

    // Use AD< AD<double> > to tape the evaluation of f(x)
    a2y[0] = f(a2x);

    // Declare a1f as the corresponding ADFun< AD<double> >
    // (make sure we do not run zero order forward during constructor)
    CppAD::ADFun<a1type> a1f;
    a1f.Dependent(a2x, a2y);

    // Values for the independent variables while taping f'(x) * v
    // Declare the independent variable for taping f'(x) * v
    // (Note we did not have to tape the creationg of a1f.)
    CppAD::Independent(a1x);

    // set the argument value x for computing f'(x) * v
    a1f.Forward(0, a1x);
    // compute f'(x) * v
    for(j = 0; j < n; j++)
        a1v[j] = double(n - j);
    a1dy = a1f.Forward(1, a1v);

    // declare g as ADFun<double> function corresponding to f'(x) * v
    CppAD::ADFun<double> g;
    g.Dependent(a1x, a1dy);

    // optimize out operations not necessary for function f'(x) * v
    g.optimize();

    // Evaluate f'(x) * v
    g.Forward(0, x);

    // compute the d/dx of f'(x) * v = f''(x) * v = v
    CPPAD_TESTVECTOR(double) w(1);
    CPPAD_TESTVECTOR(double) dw(n);
    w[0] = 1.;
    dw   = g.Reverse(1, w);

    for(j = 0; j < n; j++)
        ok &= CppAD::NearEqual(dw[j], a1v[j], eps, eps);

    return ok;
}
// END C++
