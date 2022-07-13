#ifndef CPPAD_CG_TEST_LOG1P_INCLUDED
#define CPPAD_CG_TEST_LOG1P_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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
CppAD::ADFun<T>* Log1pTestFunc(const std::vector<CppAD::AD<T> >& x) {
    using CppAD::log1p;
    using namespace CppAD;

    assert(x.size() == 1);

    // dependent variable vector
    std::vector< AD<T> > y(2);

    y[0] = log1p(x[0]);

    // define f : x -> y and vectors for derivative calculations
    return new ADFun<T> (x, y);
}

#endif