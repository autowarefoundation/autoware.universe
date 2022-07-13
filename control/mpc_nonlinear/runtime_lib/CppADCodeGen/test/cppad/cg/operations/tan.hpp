#ifndef CPPAD_CG_TEST_TAN_INCLUDED
#define CPPAD_CG_TEST_TAN_INCLUDED
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
CppAD::ADFun<T>* tanFirstFunc(const std::vector<CppAD::AD<T> >& ax) {
    using CppAD::AD;

    assert(ax.size() == 1);

    // dependent variable vector and indices
    std::vector< AD<T> > ay(1);

    ay[0] = atan(tan(ax[0]));

    // create f: x -> y and vectors used for derivative calculations
    return new CppAD::ADFun<T > (ax, ay);
}

template<class T>
CppAD::ADFun<T>* tanLastFunc(const std::vector<CppAD::AD<T> >& ax) {
    using CppAD::AD;

    assert(ax.size() == 1);

    // dependent variable vector and indices
    std::vector< AD<T> > ay(1);

    ay[0] = tan(atan(ax[0]));

    // create f: x -> y and vectors used for derivative calculations
    return new CppAD::ADFun<T > (ax, ay);
}

template<class T>
CppAD::ADFun<T>* tanhFirstFunc(const std::vector<CppAD::AD<T> >& ax) {
    using CppAD::AD;

    // independent variable vector, indices, values, and declaration
    assert(ax.size() == 1);

    // dependent variable vector and indices
    std::vector< AD<T> > ay(1);
    AD<T> z = tanh(ax[0]);
    ay[0] = .5 * log((1. + z) / (1. - z));

    // create f: x -> y and vectors used for derivative calculations
    return new CppAD::ADFun<T > (ax, ay);
}

template<class T>
CppAD::ADFun<T>* tanhLastFunc(const std::vector<CppAD::AD<T> >& ax) {
    using CppAD::AD;

    // independent variable vector, indices, values, and declaration
    assert(ax.size() == 1);

    // dependent variable vector and indices
    std::vector< AD<T> > ay(1);
    AD<T> z = .5 * log((1. + ax[0]) / (1. - ax[0]));
    ay[0] = tanh(z);

    // create f: x -> y and vectors used for derivative calculations
    return new CppAD::ADFun<T > (ax, ay);
}


#endif