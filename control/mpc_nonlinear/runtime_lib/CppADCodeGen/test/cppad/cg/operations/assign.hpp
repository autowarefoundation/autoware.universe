#ifndef CPPAD_CG_TEST_ASSIGN_INCLUDED
#define CPPAD_CG_TEST_ASSIGN_INCLUDED
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
CppAD::ADFun<T>* AssignFunc(const std::vector<CppAD::AD<T> >& u) {
    using namespace CppAD;
    using namespace std;

    assert(u.size() == 2);

    std::vector<AD<T> > w(2);
    AD<T> a = u[0] + 2.0;
    AD<T> b = u[1] + 1.0;
    b = a;
    a += 5.0;
    w[0] = a;
    w[1] = b;

    // f(v) = |w|
    return new CppAD::ADFun<T > (u, w);
}

#endif