#ifndef CPPAD_CG_TEST_ABS_INCLUDED
#define CPPAD_CG_TEST_ABS_INCLUDED
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
CppAD::ADFun<T>* AbsFunc(const std::vector<CppAD::AD<T> >& u) {
    using namespace CppAD;
    using namespace std;

    assert(u.size() == 1);

    std::vector<CppAD::AD<T> > w(1);
    w[0] = CppAD::abs(u[0]);

    // f(v) = |w|
    return new CppAD::ADFun<T>(u, w);
}

#endif