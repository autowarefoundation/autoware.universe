#ifndef CPPAD_CG_TEST_PARAMETER_INCLUDED
#define CPPAD_CG_TEST_PARAMETER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

#include <assert.h>

template<class T>
CppAD::ADFun<T>* ParameterFunc(const std::vector<CppAD::AD<T> >& ax) {
    using namespace CppAD;

    // number of different parameter values
    size_t n_parameter = 7;

    // number of parameter repeats
    size_t n_repeat = 5;

    // independent variable vector
    size_t n = ax.size();
    assert(n == n_parameter * n_repeat);

    // dependent variable vector and indices
    size_t i, m = n;
    std::vector< AD<T> > ay(m);
    for (i = 0; i < m; i++) { // must avoid Float(k) = 0 because it would get optimized out	
        size_t k = (i % n_parameter);
        k = k * k * 10 + 1;
        ay[i] = ax[i] + T(k);
    }

    // create f: ax -> ay 
    return new ADFun<T > (ax, ay);
}

#endif