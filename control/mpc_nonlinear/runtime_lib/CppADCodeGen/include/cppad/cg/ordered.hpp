#ifndef CPPAD_CG_ORDERED_INCLUDED
#define CPPAD_CG_ORDERED_INCLUDED
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
bool GreaterThanZero(const cg::CG<Base> &x) {
    if (!x.isParameter()) {
        throw cg::CGException("GreaterThanZero cannot be called for non-parameters");
    }

    return GreaterThanZero(x.getValue());
}

template<class Base>
bool GreaterThanOrZero(const cg::CG<Base> &x) {
    if (!x.isParameter()) {
        throw cg::CGException("GreaterThanOrZero cannot be called for non-parameters");
    }

    return GreaterThanOrZero(x.getValue());
}

template<class Base>
bool LessThanZero(const cg::CG<Base> &x) {
    if (!x.isParameter()) {
        throw cg::CGException("LessThanZero cannot be called for non-parameters");
    }

    return LessThanZero(x.getValue());
}

template<class Base>
bool LessThanOrZero(const cg::CG<Base> &x) {
    if (!x.isParameter()) {
        throw cg::CGException("LessThanOrZero cannot be called for non-parameters");
    }

    return LessThanOrZero(x.getValue());
}

template<class Base>
bool abs_geq(const cg::CG<Base>& x,
             const cg::CG<Base>& y) {
    if (!x.isParameter()) {
        throw cg::CGException("abs_geq cannot be called for non-parameters (x)");
    } else if (!y.isParameter()) {
        throw cg::CGException("abs_geq cannot be called for non-parameters (y)");
    }

    return abs_geq(x.getValue(), y.getValue());
}

} // END CppAD namespace

#endif