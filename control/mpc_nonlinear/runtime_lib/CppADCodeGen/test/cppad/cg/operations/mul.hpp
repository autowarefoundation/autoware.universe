#ifndef CPPAD_CG_TEST_MUL_INCLUDED
#define CPPAD_CG_TEST_MUL_INCLUDED
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
CppAD::ADFun<T>* MulTestOneFunc(const std::vector<CppAD::AD<T> >& U) {
    using namespace CppAD;

    // independent variable vector, indices, values, and declaration
    assert(U.size() == 2);

    size_t s = 0;
    size_t t = 1;

    // assign some parameters
    AD<T> zero = T(0.);
    AD<T> one = T(1.);

    // dependent variable vector and indices
    std::vector< AD<T> > Z(5);
    size_t x = 0;
    size_t y = 1;
    size_t z = 2;
    size_t u = 3;
    size_t v = 4;

    // assign the dependent variables
    Z[x] = U[s] * U[t]; // AD<double> * AD<double>
    Z[y] = Z[x] * 4.; // AD<double> *    double
    Z[z] = 4. * Z[y]; //    double  * AD<double> 
    Z[u] = one * Z[z]; // multiplication by parameter equal to one
    Z[v] = zero * Z[z]; // multiplication by parameter equal to zero

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<T > (U, Z);
}

template<class T>
CppAD::ADFun<T>* MulTestTwoFunc(const std::vector<CppAD::AD<T> >& U) {
    using namespace CppAD;

    // independent variable vector
    assert(U.size() == 1);

    AD<T> a = U[0] * 1.; // AD<double> * double
    AD<T> b = a * 2; // AD<double> * int
    AD<T> c = 3. * b; // double     * AD<double> 
    AD<T> d = 4 * c; // int        * AD<double> 

    // dependent variable vector 
    std::vector< AD<T> > Z(1);
    Z[0] = U[0] * d; // AD<double> * AD<double>

    // create f: U -> Z and vectors used for derivative calculations
    return new ADFun<T > (U, Z);
}

#endif