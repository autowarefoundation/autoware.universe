#ifndef CPPAD_CG_IDENTICAL_INCLUDED
#define CPPAD_CG_IDENTICAL_INCLUDED
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

namespace CppAD {

template<class Base>
inline bool IdenticalPar(const CppAD::cg::CG<Base>& x) {
    if (!x.isParameter()) {
        return false; // its value may change after tapping
    }
    return CppAD::IdenticalPar(x.getValue());
}

template<class Base>
inline bool IdenticalCon(const CppAD::cg::CG<Base>& x) {
    if (!x.isParameter()) {
        return false; // its value may change after tapping
    }
    return CppAD::IdenticalCon(x.getValue());
}

template<class Base>
inline bool IdenticalZero(const CppAD::cg::CG<Base>& x) {
    if (!x.isParameter()) {
        return false; // its value may change after tapping
    }
    return CppAD::IdenticalZero(x.getValue());
}

template<class Base>
inline bool IdenticalOne(const CppAD::cg::CG<Base>& x) {
    if (!x.isParameter()) {
        return false; // its value may change after tapping
    }
    return CppAD::IdenticalOne(x.getValue());
}

template<class Base>
inline bool IdenticalEqualPar(const CppAD::cg::CG<Base>& x,
                              const CppAD::cg::CG<Base>& y) {
    return x.isParameter() && y.isParameter() && CppAD::IdenticalEqualPar(x.getValue(), y.getValue());
}

template<class Base>
inline bool IdenticalEqualCon(const CppAD::cg::CG<Base>& x,
                              const CppAD::cg::CG<Base>& y) {
    return x.isParameter() && y.isParameter() && CppAD::IdenticalEqualCon(x.getValue(), y.getValue());
}

} // END CppAD namespace

#endif
