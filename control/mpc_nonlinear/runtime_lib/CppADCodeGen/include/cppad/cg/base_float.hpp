#ifndef CPPAD_CG_BASE_FLOAT_INCLUDED
#define CPPAD_CG_BASE_FLOAT_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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

/**
 * Specialization of the numeric_limits for floats
 */
template <>
class numeric_limits<cg::CG<float> > {
public:

    static cg::CG<float> epsilon() {
        return std::numeric_limits<float>::epsilon();
    }

    static cg::CG<float> min() {
        return std::numeric_limits<float>::min();
    }

    static cg::CG<float> max() {
        return std::numeric_limits<float>::max();
    }

    static cg::CG<float> quiet_NaN() {
        return std::numeric_limits<float>::quiet_NaN();
    }

    static const int digits10 
        = std::numeric_limits<float>::digits10;
};

/**
 * Specialization of the machine epsilon for CG<float>
 */
template <>
inline cg::CG<float> epsilon<cg::CG<float> >() {
    return std::numeric_limits<float>::epsilon();
}

/**
 * Absolute Zero multiplication
 */
inline cg::CG<float> azmul(const cg::CG<float>& left,
                           const cg::CG<float>& right) {
    cg::CG<float> zero(0.0);
    if (left == zero)
        return zero;
    return left * right;
}
// CPPAD_AZMUL(cg::CG<float>)

} // END CppAD namespace

#endif

