#ifndef CPPAD_CG_TEST_ACOSH_INCLUDED
#define CPPAD_CG_TEST_ACOSH_INCLUDED
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
CppAD::ADFun<T>* AcoshFunc(const std::vector<CppAD::AD<T> >& x) {
    using namespace CppAD;
    using CppAD::cosh;
    using CppAD::acosh;

    assert(x.size() == 1);

    // dependent variable vector 
    std::vector< AD<T> > y(1);
    
    auto v = cosh(x[0]);
    
    y[0] = acosh(v);

    // create f: x -> y and vectors used for derivative calculations
    return new ADFun<T>(x, y);
}

#endif