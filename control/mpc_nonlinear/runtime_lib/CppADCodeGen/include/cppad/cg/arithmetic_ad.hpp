#ifndef CPPAD_CG_ARITHMETIC_AD_INCLUDED
#define CPPAD_CG_ARITHMETIC_AD_INCLUDED
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

/*******************************************************************************
 *                 Operations with AD (resolves ambiguity)
 ******************************************************************************/
template<class Base>
inline AD<CG<Base> > operator+(const CG<Base>& left, const AD<CG<Base> >& right) {
    return CppAD::operator+(left, right);
}

template<class Base>
inline AD<CG<Base> > operator+(const AD<CG<Base> >& left, const CG<Base>& right) {
    return CppAD::operator+(left, right);
}

template<class Base>
inline AD<CG<Base> > operator-(const CG<Base>& left, const AD<CG<Base> >& right) {
    return CppAD::operator-(left, right);
}

template<class Base>
inline AD<CG<Base> > operator-(const AD<CG<Base> >& left, const CG<Base>& right) {
    return CppAD::operator-(left, right);
}

template<class Base>
inline AD<CG<Base> > operator/(const CG<Base>& left, const AD<CG<Base> >& right) {
    return CppAD::operator/(left, right);
}

template<class Base>
inline AD<CG<Base> > operator/(const AD<CG<Base> >& left, const CG<Base>& right) {
    return CppAD::operator/(left, right);
}

template<class Base>
inline AD<CG<Base> > operator*(const CG<Base>& left, const AD<CG<Base> >& right) {
    return CppAD::operator*(left, right);
}

template<class Base>
inline AD<CG<Base> > operator*(const AD<CG<Base> >& left, const CG<Base>& right) {
    return CppAD::operator*(left, right);
}

} // END cg namespace
} // END CppAD namespace

#endif

