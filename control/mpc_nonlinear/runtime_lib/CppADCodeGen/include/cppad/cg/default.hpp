#ifndef CPPAD_CG_DEFAULT_INCLUDED
#define CPPAD_CG_DEFAULT_INCLUDED
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

/**
 * Creates a parameter with a zero value
 */
template <class Base>
inline CG<Base>::CG() :
    node_(nullptr),
    value_(new Base(0.0)) {
}

template <class Base>
inline CG<Base>::CG(OperationNode<Base>& node) :
    node_(&node) {
}

template <class Base>
inline CG<Base>::CG(const Argument<Base>& arg) :
    node_(arg.getOperation()),
    value_(arg.getParameter() != nullptr ? new Base(*arg.getParameter()) : nullptr) {

}

/**
 * Creates a parameter with the given value
 */
template <class Base>
inline CG<Base>::CG(const Base &b) :
    node_(nullptr),
    value_(new Base(b)) {
}

/**
 * Copy constructor
 */
template <class Base>
inline CG<Base>::CG(const CG<Base>& orig) :
    node_(orig.node_),
    value_(orig.value_ != nullptr ? new Base(*orig.value_) : nullptr) {
}

/**
 * Move constructor
 */
template <class Base>
inline CG<Base>::CG(CG<Base>&& orig):
        node_(orig.node_),
        value_(std::move(orig.value_)) {
}

/**
 * Creates a parameter with the given value
 */
template <class Base>
inline CG<Base>& CG<Base>::operator=(const Base& b) {
    node_ = nullptr;
    if (value_ != nullptr) {
        *value_ = b;
    } else {
        value_.reset(new Base(b)); // to replace with value_ = std::make_unique once c++14 is used
    }
    return *this;
}

template <class Base>
inline CG<Base>& CG<Base>::operator=(const CG<Base>& rhs) {
    if (&rhs == this) {
        return *this;
    }
    node_ = rhs.node_;
    if (rhs.value_ != nullptr) {
        if (value_ != nullptr) {
            *value_ = *rhs.value_;
        } else {
            value_.reset(new Base(*rhs.value_)); // to replace with value_ = std::make_unique once c++14 is used
        }
    } else {
        value_.reset();
    }

    return *this;
}

template <class Base>
inline CG<Base>& CG<Base>::operator=(CG<Base>&& rhs) {
    assert(this != &rhs);

    node_ = rhs.node_;

    // steal the value
    value_ = std::move(rhs.value_);

    return *this;
}

template <class Base>
CG<Base>::~CG() = default;

} // END cg namespace
} // END CppAD namespace

#endif
