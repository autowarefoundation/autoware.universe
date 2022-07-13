#ifndef CPPAD_CG_TEST_COND_EXP_INCLUDED
#define CPPAD_CG_TEST_COND_EXP_INCLUDED
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
CppAD::ADFun<T>* CondExp_pvvvFunc(const std::vector<CppAD::AD<T> >& X) {
    using namespace CppAD;
    using namespace std;

    assert(X.size() == 3);

    // parameter value
    AD<T> one = T(1.);

    // dependent variable vector 
    std::vector< AD<T> > Y(5);

    // CondExp(parameter, variable, variable, variable)
    Y[0] = CondExpLt(one, X[0], X[1], X[2]);
    Y[1] = CondExpLe(one, X[0], X[1], X[2]);
    Y[2] = CondExpEq(one, X[0], X[1], X[2]);
    Y[3] = CondExpGe(one, X[0], X[1], X[2]);
    Y[4] = CondExpGt(one, X[0], X[1], X[2]);

    // create f: X -> Y 
    ADFun<T>* fun = new ADFun<T> (X, Y);
    // The option 'no_conditional_skip' is essential in order to avoid an 
    // assertion failure in CppAD and so that branches are not skipped with NDEBUG defined
    fun->optimize();

    return fun;
}

template<class T>
CppAD::ADFun<T>* CondExp_vpvvFunc(const std::vector<CppAD::AD<T> >& X) {
    using namespace CppAD;
    using namespace std;

    assert(X.size() == 3);

    // parameter value
    AD<T> one = T(1.);

    // dependent variable vector 
    std::vector< AD<T> > Y(5);

    // CondExp(variable, parameter, variable, variable)
    Y[0] = CondExpLt(X[0], one, X[1], X[2]);
    Y[1] = CondExpLe(X[0], one, X[1], X[2]);
    Y[2] = CondExpEq(X[0], one, X[1], X[2]);
    Y[3] = CondExpGe(X[0], one, X[1], X[2]);
    Y[4] = CondExpGt(X[0], one, X[1], X[2]);

    // create f: X -> Y 
    return new ADFun<T> (X, Y);
}

template<class T>
CppAD::ADFun<T>* CondExp_vpvpFunc(const std::vector<CppAD::AD<T> >& X) {
    using namespace CppAD;
    using namespace std;

    assert(X.size() == 3);

    // parameter value
    AD<T> one = T(1.);
    AD<T> zero = T(0.);

    // dependent variable vector 
    std::vector< AD<T> > Y(5);

    // CondExp(variable, parameter, variable, variable)
    Y[0] = CondExpLt(X[0], one, X[1] * X[2], zero);
    Y[1] = CondExpLe(X[0], one, X[1] * X[2], zero);
    Y[2] = CondExpEq(X[0], one, X[1] * X[2], zero);
    Y[3] = CondExpGe(X[0], one, X[1] * X[2], zero);
    Y[4] = CondExpGt(X[0], one, X[1] * X[2], zero);

    // create f: X -> Y 
    ADFun<T>* fun = new ADFun<T> (X, Y);
    // The option 'no_conditional_skip' is essential in order to avoid an 
    // assertion failure in CppAD and so that branches are not skipped with NDEBUG defined
    fun->optimize();

    return fun;
}

template<class T>
CppAD::ADFun<T>* CondExp_vvpvFunc(const std::vector<CppAD::AD<T> >& X) {
    using namespace CppAD;
    using namespace std;

    assert(X.size() == 3);

    // parameter value
    AD<T> three = T(3.);

    // dependent variable vector 
    std::vector< AD<T> > Y(5);

    // CondExp(variable, variable, parameter, variable)
    Y[0] = CondExpLt(X[0], X[1], three, X[2]);
    Y[1] = CondExpLe(X[0], X[1], three, X[2]);
    Y[2] = CondExpEq(X[0], X[1], three, X[2]);
    Y[3] = CondExpGe(X[0], X[1], three, X[2]);
    Y[4] = CondExpGt(X[0], X[1], three, X[2]);

    // create f: X -> Y 
    return new ADFun<T> (X, Y);
}

template<class T>
CppAD::ADFun<T>* CondExp_vvvpFunc(const std::vector<CppAD::AD<T> >& X) {
    using namespace CppAD;
    using namespace std;

    assert(X.size() == 3);

    // parameter value
    AD<T> three = T(3.);

    // dependent variable vector 
    std::vector< AD<T> > Y(5);

    // CondExp(variable, variable, variable, parameter)
    Y[0] = CondExpLt(X[0], X[1], X[2], three);
    Y[1] = CondExpLe(X[0], X[1], X[2], three);
    Y[2] = CondExpEq(X[0], X[1], X[2], three);
    Y[3] = CondExpGe(X[0], X[1], X[2], three);
    Y[4] = CondExpGt(X[0], X[1], X[2], three);

    // create f: X -> Y 
    ADFun<T>* fun = new ADFun<T> (X, Y);
    // The option 'no_conditional_skip' is essential in order to avoid an 
    // assertion failure in CppAD and so that branches are not skipped with NDEBUG defined
    fun->optimize();

    return fun;
}

#endif