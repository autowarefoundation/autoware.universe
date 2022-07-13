#ifndef CPPAD_CG_MATH_OTHER_INCLUDED
#define CPPAD_CG_MATH_OTHER_INCLUDED
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

template <class Base>
inline CppAD::cg::CG<Base> pow(const CppAD::cg::CG<Base>& x,
                               const CppAD::cg::CG<Base>& y) {
    using namespace CppAD::cg;

    if (x.isParameter() && y.isParameter()) {
        return CG<Base> (pow(x.getValue(), y.getValue()));
    }

    CodeHandler<Base>* handler;
    if (y.isParameter()) {
        if (y.isIdenticalZero()) {
            return CG<Base> (Base(1.0)); // does not consider that x could be infinity
        } else if (y.isIdenticalOne()) {
            return CG<Base> (x);
        }
        handler = x.getCodeHandler();
    } else {
        handler = y.getCodeHandler();
    }

    CG<Base> result(*handler->makeNode(CGOpCode::Pow,{x.argument(), y.argument()}));
    if (x.isValueDefined() && y.isValueDefined()) {
        result.setValue(pow(x.getValue(), y.getValue()));
    }
    return result;
}

/*******************************************************************************
 *                          pow() with other types
 ******************************************************************************/

template <class Base>
inline CppAD::cg::CG<Base> pow(const Base& x,
                               const CppAD::cg::CG<Base>& y) {
    return CppAD::pow<Base>(CppAD::cg::CG<Base>(x), y);
}

template <class Base>
inline CppAD::cg::CG<Base> pow(const CppAD::cg::CG<Base>& x,
                               const Base& y) {
    return CppAD::pow<Base>(x, CppAD::cg::CG<Base>(y));
}

template <class Base>
CppAD::cg::CG<Base> pow(const int& x,
                        const CppAD::cg::CG<Base>& y) {
    return pow(CppAD::cg::CG<Base>(x), y);
}

/*******************************************************************************
 * 
 ******************************************************************************/
template <class Base>
inline CppAD::cg::CG<Base> sign(const CppAD::cg::CG<Base>& x) {
    using namespace CppAD::cg;

    if (x.isParameter()) {
        if (x.getValue() > Base(0.0)) {
            return CG<Base> (Base(1.0));
        } else if (x.getValue() == Base(0.0)) {
            return CG<Base> (Base(0.0));
        } else {
            return CG<Base> (Base(-1.0));
        }
    }

    CodeHandler<Base>& h = *x.getOperationNode()->getCodeHandler();
    CG<Base> result(*h.makeNode(CGOpCode::Sign, x.argument()));
    if (x.isValueDefined()) {
        if (x.getValue() > Base(0.0)) {
            result.setValue(Base(1.0));
        } else if (x.getValue() == Base(0.0)) {
            result.setValue(Base(0.0));
        } else {
            result.setValue(Base(-1.0));
        }
    }
    return result;
}

} // END CppAD namespace

#endif