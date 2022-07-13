#ifndef CPPAD_CG_ARITHMETIC_INCLUDED
#define CPPAD_CG_ARITHMETIC_INCLUDED
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
CodeHandler<Base>* getHandler(const CG<Base>& left,
                              const CG<Base>& right) {

    CPPADCG_ASSERT_UNKNOWN(!left.isParameter() || !right.isParameter());

    CodeHandler<Base>* lh = left.getCodeHandler();
    CodeHandler<Base>* rh = right.getCodeHandler();

    if (lh == nullptr) {
        return rh;
    } else if (rh == nullptr) {
        return lh;
    } else {
        if (lh != rh) {
            throw CGException("Attempting to use several source code generation handlers in the same source code generation");
        }
        return lh;
    }
}

template<class Base>
inline CG<Base> operator+(const CG<Base>& left, const CG<Base>& right) {
    if (left.isParameter() && right.isParameter()) {
        return CG<Base> (left.getValue() + right.getValue());

    } else {
        if (left.isParameter()) {
            if (left.isIdenticalZero()) {
                return right;
            }
        } else if (right.isParameter()) {
            if (right.isIdenticalZero()) {
                return left;
            }
        }

        CodeHandler<Base>* handler = getHandler(left, right);

        CG<Base> result(*handler->makeNode(CGOpCode::Add,{left.argument(), right.argument()}));
        if (left.isValueDefined() && right.isValueDefined()) {
            result.setValue(left.getValue() + right.getValue());
        }
        return result;
    }
}

template<class Base>
inline CG<Base> operator-(const CG<Base>& left, const CG<Base>& right) {
    if (left.isParameter() && right.isParameter()) {
        return CG<Base> (left.getValue() - right.getValue());

    } else {
        if (right.isParameter()) {
            if (right.isIdenticalZero()) {
                return left;
            }
        }

        CodeHandler<Base>* handler = getHandler(left, right);

        CG<Base> result(*handler->makeNode(CGOpCode::Sub,{left.argument(), right.argument()}));
        if (left.isValueDefined() && right.isValueDefined()) {
            result.setValue(left.getValue() - right.getValue());
        }
        return result;
    }
}

template<class Base>
inline CG<Base> operator*(const CG<Base>& left, const CG<Base>& right) {
    if (left.isParameter() && right.isParameter()) {
        return CG<Base> (left.getValue() * right.getValue());

    } else {
        if (left.isParameter()) {
            if (left.isIdenticalZero()) {
                return CG<Base> (Base(0.0)); // does not consider the possibility of right being infinity
            } else if (left.isIdenticalOne()) {
                return right;
            }
        } else if (right.isParameter()) {
            if (right.isIdenticalZero()) {
                return CG<Base> (Base(0.0)); // does not consider the possibility of left being infinity
            } else if (right.isIdenticalOne()) {
                return left;
            }
        }

        CodeHandler<Base>* handler = getHandler(left, right);

        CG<Base> result(*handler->makeNode(CGOpCode::Mul,{left.argument(), right.argument()}));
        if (left.isValueDefined() && right.isValueDefined()) {
            result.setValue(left.getValue() * right.getValue());
        }
        return result;
    }
}

template<class Base>
inline CG<Base> operator/(const CG<Base>& left, const CG<Base>& right) {
    if (left.isParameter() && right.isParameter()) {
        return CG<Base> (left.getValue() / right.getValue());

    } else {
        if (left.isParameter()) {
            if (left.isIdenticalZero()) {
                return CG<Base> (Base(0.0)); // does not consider the possibility of right being infinity or zero
            }
        } else if (right.isParameter()) {
            if (right.isIdenticalOne()) {
                return left;
            }
        } else if (left.getOperationNode() == right.getOperationNode()) {
            return CG<Base>(Base(1.0)); // does not consider the possibility of left/right being infinity or zero
        }

        CodeHandler<Base>* handler = getHandler(left, right);

        CG<Base> result(*handler->makeNode(CGOpCode::Div,{left.argument(), right.argument()}));
        if (left.isValueDefined() && right.isValueDefined()) {
            result.setValue(left.getValue() / right.getValue());
        }
        return result;
    }
}

template<class Base>
inline CG<Base> operator+(const Base& left, const CG<Base>& right) {
    return CG<Base>(left) + right;
}

template<class Base>
inline CG<Base> operator+(const CG<Base>& left, const Base& right) {
    return left + CG<Base>(right);
}

template<class Base>
inline CG<Base> operator-(const Base& left, const CG<Base>& right) {
    return CG<Base>(left) - right;
}

template<class Base>
inline CG<Base> operator-(const CG<Base>& left, const Base& right) {
    return left - CG<Base>(right);
}

template<class Base>
inline CG<Base> operator/(const Base& left, const CG<Base>& right) {
    return CG<Base>(left) / right;
}

template<class Base>
inline CG<Base> operator/(const CG<Base>& left, const Base& right) {
    return left / CG<Base>(right);
}

template<class Base>
inline CG<Base> operator*(const Base& left, const CG<Base>& right) {
    return CG<Base>(left) * right;
}

template<class Base>
inline CG<Base> operator*(const CG<Base>& left, const Base& right) {
    return left * CG<Base>(right);
}

/*******************************************************************************
 *                        Operations with other types
 ******************************************************************************/

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator+(const T& left, const CG<Base>& right) {
    return CG<Base>(Base(left)) + right;
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator+(const CG<Base>& left, const T& right) {
    return left + CG<Base>(Base(right));
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator-(const T& left, const CG<Base>& right) {
    return CG<Base>(Base(left)) - right;
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator-(const CG<Base>& left, const T& right) {
    return left - CG<Base>(Base(right));
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator/(const T& left, const CG<Base>& right) {
    return CG<Base>(Base(left)) / right;
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator/(const CG<Base>& left, const T& right) {
    return left / CG<Base>(Base(right));
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator*(const T& left, const CG<Base>& right) {
    return CG<Base>(Base(left)) * right;
}

template<class Base, class T>
inline
typename std::enable_if<std::is_constructible<Base, const T&>::value, CG<Base> >::type
operator*(const CG<Base>& left, const T& right) {
    return left * CG<Base>(Base(right));
}

} // END cg namespace
} // END CppAD namespace

#endif

