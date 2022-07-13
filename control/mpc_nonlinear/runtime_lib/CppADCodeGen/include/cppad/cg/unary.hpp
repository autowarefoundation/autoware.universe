#ifndef CPPAD_CG_UNARY_INCLUDED
#define CPPAD_CG_UNARY_INCLUDED
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
namespace cg {

template<class Base>
inline CG<Base> CG<Base>::operator+() const {
    return CG<Base> (*this); // nothing to do
}

template<class Base>
inline CG<Base> CG<Base>::operator-() const {
    if (isParameter()) {
        return CG<Base> (-getValue());

    } else {
        CodeHandler<Base>& h = *getCodeHandler();
        CG<Base> result(*h.makeNode(CGOpCode::UnMinus, this->argument()));
        if (isValueDefined()) {
            result.setValue(-getValue());
        }
        return result;
    }
}

} // END cg namespace
} // END CppAD namespace

#endif