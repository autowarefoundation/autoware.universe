#ifndef CPPAD_CG_VARIABLE_INCLUDED
#define CPPAD_CG_VARIABLE_INCLUDED
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
inline CodeHandler<Base>* CG<Base>::getCodeHandler() const {
    if (node_ != nullptr)
        return node_->getCodeHandler();
    else
        return nullptr;
}

template<class Base>
inline bool CG<Base>::isVariable() const {
    return node_ != nullptr;
}

template<class Base>
inline bool CG<Base>::isParameter() const {
    return node_ == nullptr;
}

template<class Base>
inline bool CG<Base>::isValueDefined() const {
    return value_ != nullptr;
}

template<class Base>
inline const Base& CG<Base>::getValue() const {
    if (!isValueDefined()) {
        throw CGException("No value defined for this variable");
    }

    return *value_;
}

template<class Base>
inline void CG<Base>::setValue(const Base& b) {
    if (value_ != nullptr) {
        *value_ = b;
    } else {
        value_.reset(new Base(b)); // to replace with value_ = std::make_unique once c++14 is used
    }
}

template<class Base>
inline bool CG<Base>::isIdenticalZero() const {
    return isParameter() && CppAD::IdenticalZero(getValue());
}

template<class Base>
inline bool CG<Base>::isIdenticalOne() const {
    return isParameter() && CppAD::IdenticalOne(getValue());
}

template<class Base>
inline void CG<Base>::makeParameter(const Base &b) {
    node_ = nullptr;
    setValue(b);
}

template<class Base>
inline void CG<Base>::makeVariable(OperationNode<Base>& operation) {
    node_ = &operation;
    value_.reset();
}

template<class Base>
inline void CG<Base>::makeVariable(OperationNode<Base>& operation,
                                   std::unique_ptr<Base>& value) {
    node_ = &operation;
    value_ = std::move(value);
}

template<class Base>
inline OperationNode<Base>* CG<Base>::getOperationNode() const {
    return node_;
}

template<class Base>
inline Argument<Base> CG<Base>::argument() const {
    if (node_ != nullptr)
        return Argument<Base> (*node_);
    else
        return Argument<Base> (*value_);
}

} // END cg namespace
} // END CppAD namespace

#endif
