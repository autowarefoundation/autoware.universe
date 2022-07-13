#ifndef CPPAD_CG_TEST_ADD_INCLUDED
#define CPPAD_CG_TEST_ADD_INCLUDED
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
CppAD::ADFun<T>* AddFunc(const std::vector<CppAD::AD<T> >& x) {
    using namespace CppAD;
    using namespace std;

    assert(x.size() == 2);

    // dependent variable vector and indices
    std::vector<AD<T> > y(4);

    // dependent variable values
    y[0] = x[0] + x[1]; // AD<double> + AD<double>
    y[1] = y[0] + 1.; // AD<double> + double
    y[2] = 1. + y[1]; // double + AD<double>
    y[2] = 1. + y[1]; // double + AD<double> 
    y[3] = y[1] + (-1.); // AD<double> + (-double)

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<T>(x, y);
}

#endif

