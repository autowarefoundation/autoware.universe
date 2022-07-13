#ifndef CPPAD_CG_TEST_ASIN_INCLUDED
#define CPPAD_CG_TEST_ASIN_INCLUDED
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
CppAD::ADFun<T>* AsinFunc(const std::vector<CppAD::AD<T> >& u) {
    using namespace CppAD;
    using namespace std;
    using CppAD::asin;

    // a temporary values
    AD<T> x = sin(u[0]);

    // dependent variable vector 
    std::vector< AD<T> > Z(1);
    Z[0] = asin(x); // asin( sin(u) )

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<T>(u, Z);
}

#endif