#ifndef CPPAD_CG_TEST_UNARY_INCLUDED
#define CPPAD_CG_TEST_UNARY_INCLUDED
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
CppAD::ADFun<T>* UnaryPlusFunc(const std::vector<CppAD::AD<T> >& U) {
    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    assert(U.size() == 2);

    // dependent variable vector and indices
    std::vector< AD<T> > Z(2);

    // dependent variable values
    Z[0] = +U[0]; // + AD<double>
    Z[1] = +U[1]; // + AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<T > (U, Z);
}

template<class T>
CppAD::ADFun<T>* UnaryMinusFunc(const std::vector<CppAD::AD<T> >& U) {
    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    assert(U.size() == 2);

    // dependent variable vector and indices
    std::vector< AD<T> > Z(2);

    // dependent variable values
    Z[0] = -U[0]; // + AD<double>
    Z[1] = -U[1]; // + AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<T > (U, Z);
}


#endif
