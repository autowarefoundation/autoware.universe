#ifndef CPPAD_CG_COMPARE_INCLUDED
#define CPPAD_CG_COMPARE_INCLUDED
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
inline bool operator ==(const CG<Base>& left, const CG<Base>& right) {
    if (left.isParameter() && right.isParameter()) {
        return left.getValue() == right.getValue();
    } else if (left.isParameter() || right.isParameter()) {
        return false;
    } else {
        return left.getOperationNode() == right.getOperationNode();
    }
}

template<class Base>
inline bool operator !=(const CG<Base>& left, const CG<Base>& right) {
    if (left.isParameter() && right.isParameter()) {
        return left.getValue() != right.getValue();
    } else if (left.isParameter() || right.isParameter()) {
        return true;
    } else {
        return left.getOperationNode() != right.getOperationNode();
    }
}

#define CPPAD_CG_OPERATOR(Op)                                                  \
    template<class Base>                                                       \
    inline bool operator Op(const CG<Base> &left, const CG<Base> &right) {     \
        if (left.isParameter() && right.isParameter()) {                       \
            return left.getValue() Op right.getValue();                        \
        } else {                                                               \
            throw CGException("Cannot use the "#Op" comparison operator in non parameter variables");\
        }                                                                      \
    }

CPPAD_CG_OPERATOR(>)
CPPAD_CG_OPERATOR( >=)
CPPAD_CG_OPERATOR(<)
CPPAD_CG_OPERATOR( <=)

template<class Base>
inline bool operator==(const CG<Base>& left, const Base& right) {
    if (left.isParameter()) {
        return left.getValue() == right;
    } else {
        return false;
    }
}

template<class Base>
inline bool operator==(const Base& left, const CG<Base>& right) {
    if (right.isParameter()) {
        return left == right.getValue();
    } else {
        return false;
    }
}

template<class Base>
inline bool operator!=(const CG<Base>& left, Base right) {
    if (left.isParameter()) {
        return left.getValue() != right;
    } else {
        return true;
    }
}

template<class Base>
inline bool operator!=(const Base& left, const CG<Base>& right) {
    if (right.isParameter()) {
        return left != right.getValue();
    } else {
        return true;
    }
}

// comparison with double (required by CppAD SparseHessian)
template<class Base>
inline bool operator!=(const CG<Base> &left, double right) {
    if (left.isParameter()) {
        return left.getValue() != right;
    } else {
        return true;
    }
}

} // END cg namespace
} // END CppAD namespace

#endif

