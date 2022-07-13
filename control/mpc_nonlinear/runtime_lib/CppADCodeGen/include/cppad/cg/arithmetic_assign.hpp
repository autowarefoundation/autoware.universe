#ifndef CPPAD_CG_ARITHMETIC_ASSIGN_INCLUDED
#define CPPAD_CG_ARITHMETIC_ASSIGN_INCLUDED
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
inline CG<Base>& CG<Base>::operator+=(const CG<Base> &right) {
    if (isParameter() && right.isParameter()) {
        *value_ += *right.value_;

    } else {
        CodeHandler<Base>* handler;
        if (isParameter()) {
            if (isIdenticalZero()) {
                *this = right;
                return *this;
            }

            handler = right.node_->getCodeHandler();

        } else if (right.isParameter()) {
            if (right.isIdenticalZero()) {
                return *this; // nothing to do
            }

            handler = node_->getCodeHandler();

        } else { // both left and right hand sides are variables
            CPPADCG_ASSERT_UNKNOWN(node_->getCodeHandler() == right.node_->getCodeHandler());
            handler = node_->getCodeHandler();
        }

        std::unique_ptr<Base> value;
        if (isValueDefined() && right.isValueDefined()) {
            value.reset(new Base(getValue() + right.getValue()));
        }

        makeVariable(*handler->makeNode(CGOpCode::Add,{argument(), right.argument()}), value);
    }

    return *this;
}

template<class Base>
inline CG<Base>& CG<Base>::operator-=(const CG<Base> &right) {
    if (isParameter() && right.isParameter()) {
        *value_ -= *right.value_;

    } else {
        CodeHandler<Base>* handler;
        if (isParameter()) {
            handler = right.node_->getCodeHandler();

        } else if (right.isParameter()) {
            if (right.isIdenticalZero()) {
                return *this; // nothing to do
            }

            handler = node_->getCodeHandler();

        } else { // both left and right hand sides are variables
            CPPADCG_ASSERT_UNKNOWN(node_->getCodeHandler() == right.node_->getCodeHandler());
            handler = node_->getCodeHandler();
        }

        std::unique_ptr<Base> value;
        if (isValueDefined() && right.isValueDefined()) {
            value.reset(new Base(getValue() - right.getValue()));
        }

        makeVariable(*handler->makeNode(CGOpCode::Sub,{argument(), right.argument()}), value);
    }

    return *this;
}

template<class Base>
inline CG<Base>& CG<Base>::operator*=(const CG<Base> &right) {
    if (isParameter() && right.isParameter()) {
        *value_ *= *right.value_;

    } else {
        CodeHandler<Base>* handler;
        if (isParameter()) {
            if (isIdenticalZero()) {
                return *this; // nothing to do (does not consider that right might be infinity)
            } else if (isIdenticalOne()) {
                *this = right;
                return *this;
            }

            handler = right.node_->getCodeHandler();

        } else if (right.isParameter()) {
            if (right.isIdenticalZero()) {
                makeParameter(Base(0.0)); // does not consider that left might be infinity
                return *this;
            } else if (right.isIdenticalOne()) {
                return *this; // nothing to do
            }

            handler = node_->getCodeHandler();

        } else {  // both left and right hand sides are variables
            CPPADCG_ASSERT_UNKNOWN(node_->getCodeHandler() == right.node_->getCodeHandler());
            handler = node_->getCodeHandler();
        }

        std::unique_ptr<Base> value;
        if (isValueDefined() && right.isValueDefined()) {
            value.reset(new Base(getValue() * right.getValue()));
        }

        makeVariable(*handler->makeNode(CGOpCode::Mul,{argument(), right.argument()}), value);
    }

    return *this;
}

template<class Base>
inline CG<Base>& CG<Base>::operator/=(const CG<Base> &right) {
    if (isParameter() && right.isParameter()) {
        *value_ /= *right.value_;

    } else {
        CodeHandler<Base>* handler;
        if (isParameter()) {
            if (isIdenticalZero()) {
                return *this; // nothing to do (does not consider that right might be infinity or zero)
            }

            handler = right.node_->getCodeHandler();

        } else if (right.isParameter()) {
            if (right.isIdenticalOne()) {
                return *this; // nothing to do
            }

            handler = node_->getCodeHandler();

        } else {  // both left and right hand sides are variables
            CPPADCG_ASSERT_UNKNOWN(node_->getCodeHandler() == right.node_->getCodeHandler());
            handler = node_->getCodeHandler();
        }

        std::unique_ptr<Base> value;
        if (isValueDefined() && right.isValueDefined()) {
            value.reset(new Base(getValue() / right.getValue()));
        }

        makeVariable(*handler->makeNode(CGOpCode::Div,{argument(), right.argument()}), value);
    }

    return *this;
}

template<class Base>
inline CG<Base>& CG<Base>::operator+=(const Base &right) {
    return operator+=(CG<Base> (right));
}

template<class Base>
inline CG<Base>& CG<Base>::operator-=(const Base &right) {
    return operator-=(CG<Base> (right));
}

template<class Base>
inline CG<Base>& CG<Base>::operator/=(const Base &right) {
    return operator/=(CG<Base> (right));
}

template<class Base>
inline CG<Base>& CG<Base>::operator*=(const Base &right) {
    return operator*=(CG<Base> (right));
}

template<class Base>
template<class T>
inline CG<Base>& CG<Base>::operator+=(const T &right) {
    return operator+=(CG<Base> (right));
}

template<class Base>
template<class T>
inline CG<Base>& CG<Base>::operator-=(const T &right) {
    return operator-=(CG<Base> (right));
}

template<class Base>
template<class T>
inline CG<Base>& CG<Base>::operator/=(const T &right) {
    return operator/=(CG<Base> (right));
}

template<class Base>
template<class T>
inline CG<Base>& CG<Base>::operator*=(const T &right) {
    return operator*=(CG<Base> (right));
}

} // END cg namespace
} // END CppAD namespace

#endif

