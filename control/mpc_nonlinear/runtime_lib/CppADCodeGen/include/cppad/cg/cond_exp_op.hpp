#ifndef CPPAD_CG_COND_EXP_OP_INCLUDED
#define CPPAD_CG_COND_EXP_OP_INCLUDED
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

namespace {

/**
 * Determine if two CG objects are equal
 */
template<class Base>
inline bool isSameExpression(const cg::CG<Base>& trueCase,
                             const cg::CG<Base>& falseCase) {
    return (trueCase.isParameter() && falseCase.isParameter() &&
            trueCase.getValue() == falseCase.getValue()) ||
            (trueCase.isVariable() && falseCase.isVariable() &&
            trueCase.getOperationNode() == falseCase.getOperationNode());
}

/**
 * Get the code handler out of some CG objects
 *
 * @throws cg::CGException
 */
template<class Base>
inline cg::CodeHandler<Base>* findCodeHandler(const cg::CG<Base>& left,
                                              const cg::CG<Base>& right,
                                              const cg::CG<Base>& trueCase,
                                              const cg::CG<Base>& falseCase) {
    cg::CodeHandler<Base>* handler;

    cg::CodeHandler<Base>* lh = left.getCodeHandler();
    cg::CodeHandler<Base>* rh = right.getCodeHandler();
    cg::CodeHandler<Base>* th = trueCase.getCodeHandler();
    cg::CodeHandler<Base>* fh = falseCase.getCodeHandler();

    if (!left.isParameter()) {
        handler = lh;
    } else if (!right.isParameter()) {
        handler = rh;
    } else if (!trueCase.isParameter()) {
        handler = th;
    } else if (!falseCase.isParameter()) {
        handler = fh;
    } else {
        CPPAD_ASSERT_UNKNOWN(0)
        throw cg::CGException("Unexpected error!");
    }

    if ((!right.isParameter() && rh != handler)
            || (!trueCase.isParameter() && th != handler)
            || (!falseCase.isParameter() && fh != handler)) {
        throw cg::CGException("Attempting to use different source code generation handlers in the same source code generation");
    }

    return handler;
}

} // END namespace

/**
 * Creates a conditional expression.
 * INTERNAL ONLY
 */
template<class Base>
inline cg::CG<Base> CondExp(cg::CGOpCode op,
                            const cg::CG<Base>& left,
                            const cg::CG<Base>& right,
                            const cg::CG<Base>& trueCase,
                            const cg::CG<Base>& falseCase,
                            bool (*compare)(const Base&, const Base&)) {
    using namespace CppAD::cg;

    if (left.isParameter() && right.isParameter()) {
        if (compare(left.getValue(), right.getValue()))
            return trueCase;
        else
            return falseCase;


    } else if (isSameExpression(trueCase, falseCase)) {
        return trueCase;
    } else {

        CodeHandler<Base>* handler = findCodeHandler(left, right, trueCase, falseCase);

        CG<Base> result(*handler->makeNode(op,{left.argument(), right.argument(), trueCase.argument(), falseCase.argument()}));

        if (left.isValueDefined() && right.isValueDefined()) {
            if (compare(left.getValue(), right.getValue())) {
                if (trueCase.isValueDefined()) {
                    result.setValue(trueCase.getValue());
                }
            } else
                if (falseCase.isValueDefined()) {
                result.setValue(falseCase.getValue());
            }
        }

        return result;
    }

}

/**
 * Creates a conditional expression for "less than"
 */
template<class Base>
inline cg::CG<Base> CondExpLt(const cg::CG<Base>& left,
                              const cg::CG<Base>& right,
                              const cg::CG<Base>& trueCase,
                              const cg::CG<Base>& falseCase) {

    bool (*compare)(const Base&, const Base&) = [](const Base& l, const Base & r) {
        return l < r;
    };

    return CondExp(cg::CGOpCode::ComLt,
                   left, right,
                   trueCase, falseCase,
                   compare);
}

/**
 * Creates a conditional expression for "less or equal"
 */
template<class Base>
inline cg::CG<Base> CondExpLe(const cg::CG<Base>& left,
                              const cg::CG<Base>& right,
                              const cg::CG<Base>& trueCase,
                              const cg::CG<Base>& falseCase) {
    bool (*comp)(const Base&, const Base&) = [](const Base& l, const Base & r) {
        return l <= r;
    };

    return CondExp(cg::CGOpCode::ComLe,
                   left, right,
                   trueCase, falseCase,
                   comp);
}

/**
 * Creates a conditional expression for "equals"
 */
template<class Base>
inline cg::CG<Base> CondExpEq(const cg::CG<Base>& left,
                              const cg::CG<Base>& right,
                              const cg::CG<Base>& trueCase,
                              const cg::CG<Base>& falseCase) {
    bool (*comp)(const Base&, const Base&) = [](const Base& l, const Base & r) {
        return l == r;
    };

    return CondExp(cg::CGOpCode::ComEq,
                   left, right,
                   trueCase, falseCase,
                   comp);
}

/**
 * Creates a conditional expression for "greater or equal"
 */
template<class Base>
inline cg::CG<Base> CondExpGe(const cg::CG<Base>& left,
                              const cg::CG<Base>& right,
                              const cg::CG<Base>& trueCase,
                              const cg::CG<Base>& falseCase) {
    bool (*comp)(const Base&, const Base&) = [](const Base& l, const Base & r) {
        return l >= r;
    };

    return CondExp(cg::CGOpCode::ComGe,
                   left, right,
                   trueCase, falseCase,
                   comp);
}

/**
 * Creates a conditional expression for "greater than"
 */
template<class Base>
inline cg::CG<Base> CondExpGt(const cg::CG<Base>& left,
                              const cg::CG<Base>& right,
                              const cg::CG<Base>& trueCase,
                              const cg::CG<Base>& falseCase) {
    bool (*comp)(const Base&, const Base&) = [](const Base& l, const Base & r) {
        return l > r;
    };

    return CondExp(cg::CGOpCode::ComGt,
                   left, right,
                   trueCase, falseCase,
                   comp);
}

template<class Base>
inline cg::CG<Base> CondExpOp(enum CompareOp cop,
                              const cg::CG<Base>& left,
                              const cg::CG<Base>& right,
                              const cg::CG<Base>& trueCase,
                              const cg::CG<Base>& falseCase) {
    switch (cop) {
        case CompareLt:
            return CondExpLt(left, right, trueCase, falseCase);

        case CompareLe:
            return CondExpLe(left, right, trueCase, falseCase);

        case CompareEq:
            return CondExpEq(left, right, trueCase, falseCase);

        case CompareGe:
            return CondExpGe(left, right, trueCase, falseCase);

        case CompareGt:
            return CondExpGt(left, right, trueCase, falseCase);

        default:
            CPPAD_ASSERT_UNKNOWN(0)
            return trueCase;
    }
}

} // END CppAD namespace

#endif
