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
Old example for deprecated interface.
$spell
    CppAD
    Jac
$$

$section Computing Dependency: Example and Test$$

$head Discussion$$
The partial of an dependent variable with respect to an independent variable
might always be zero even though the dependent variable depends on the
value of the dependent variable. Consider the following case
$latex \[
f(x) = {\rm sign} (x) =
\left\{ \begin{array}{rl}
    +1 & {\rm if} \; x > 0 \\
    0  & {\rm if} \; x = 0 \\
    -1 & {\rm if} \; x < 0
\end{array} \right.
\] $$
In this case the value of $latex f(x)$$ depends on the value of $latex x$$
but CppAD always returns zero for the derivative of the $cref sign$$ function.

$head Dependency Pattern$$
If the $th i$$ dependent variables depends on the
value of the $th j$$ independent variable,
the corresponding entry in the dependency pattern is non-zero (true).
Otherwise it is zero (false).
CppAD uses $cref/sparsity patterns/glossary/Sparsity Pattern/$$
to represent dependency matrices.
The $icode dependency$$ argument to
$cref/ForSparseJac/ForSparseJac/dependency/$$ and
$cref/RevSparseJac/RevSparseJac/dependency/$$ is a flag that signals
that the dependency pattern (instead of the sparsity pattern) is computed.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/cppad.hpp>
namespace {
    double heavyside(const double& x)
    {   if( x <= 0.0 )
            return 0.0;
        return 1.0;
    }
    CPPAD_DISCRETE_FUNCTION(double, heavyside)
}

bool dependency(void)
{   bool ok = true;
    using CppAD::AD;
    using CppAD::NearEqual;

    // VecAD object for use later
    CppAD::VecAD<double> vec_ad(2);
    vec_ad[0] = 0.0;
    vec_ad[1] = 1.0;

    // domain space vector
    size_t n  = 5;
    CPPAD_TESTVECTOR(AD<double>) ax(n);
    for(size_t j = 0; j < n; j++)
        ax[j] = AD<double>(j + 1);

    // declare independent variables and start tape recording
    CppAD::Independent(ax);

    // some AD constants
    AD<double> azero(0.0), aone(1.0);

    // range space vector
    size_t m  = n;
    size_t m1 = n - 1;
    CPPAD_TESTVECTOR(AD<double>) ay(m);
    ay[m1-0] = sign( ax[0] );
    ay[m1-1] = CondExpLe( ax[1], azero, azero, aone);
    ay[m1-2] = CondExpLe( azero, ax[2], azero, aone);
    ay[m1-3] = heavyside( ax[3] );
    ay[m1-4] = vec_ad[ ax[4] - AD<double>(4.0) ];

    // create f: x -> y and stop tape recording
    CppAD::ADFun<double> f(ax, ay);

    // -----------------------------------------------------------
    // ForSparseJac and bool dependency
    bool transpose  = false;
    bool dependency;
    // could replace CppAD::vectorBooll by CPPAD_TEST_VECTOR<bool>
    CppAD::vectorBool eye_bool(n * n), depend_bool(m * n);
    for(size_t i = 0; i < n; i++)
    {   for(size_t j = 0; j < n; j++)
            eye_bool[i * n + j] = (i == j);
    }
    dependency = true;
    depend_bool = f.ForSparseJac(n, eye_bool, transpose, dependency);
    for(size_t i = 0; i < m; i++)
    {   for(size_t j = 0; j < n; j++)
            ok &= depend_bool[i * n + j] == (i == (m1-j));
    }
    dependency = false;
    depend_bool = f.ForSparseJac(n, eye_bool, transpose, dependency);
    for(size_t i = 0; i < m; i++)
    {   for(size_t j = 0; j < n; j++)
            ok &= depend_bool[i * n + j] == false;
    }

    // -----------------------------------------------------------
    // RevSparseJac and set dependency
    CppAD::vector<    std::set<size_t> > eye_set(m), depend_set(m);
    for(size_t i = 0; i < m; i++)
    {   ok &= eye_set[i].empty();
        eye_set[i].insert(i);
    }
    dependency = true;
    depend_set = f.RevSparseJac(n, eye_set, transpose, dependency);
    for(size_t i = 0; i < m; i++)
    {   std::set<size_t> check;
        check.insert(m1 - i);
        ok &= depend_set[i] == check;
    }
    dependency = false;
    depend_set = f.RevSparseJac(n, eye_set, transpose, dependency);
    for(size_t i = 0; i < m; i++)
        ok &= depend_set[i].empty();
    return ok;
}

// END C++
