#ifndef CPPAD_CG_TEST_LOG_10_INCLUDED
#define CPPAD_CG_TEST_LOG_10_INCLUDED
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
CppAD::ADFun<T>* Log10Func(const std::vector<CppAD::AD<T> >& U) {
    using CppAD::log10;
    using CppAD::log;
    using namespace CppAD;

    assert(U.size() == 1);

    // dependent variable vector, indices, and values
    std::vector< AD<T> > Z(2);
    size_t x = 0;
    size_t y = 1;
    Z[x] = log10(U[0]);
    Z[y] = log10(Z[x]);

    // define f : U -> Z and vectors for derivative calculations
    return new ADFun<T > (U, Z);
}

#endif