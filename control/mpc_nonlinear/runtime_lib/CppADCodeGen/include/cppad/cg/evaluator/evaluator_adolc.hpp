#ifndef CPPAD_CG_EVALUATOR_ADOLC_INCLUDED
#define CPPAD_CG_EVALUATOR_ADOLC_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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

#include <adolc/adolc.h>
#include <adolc/drivers/drivers.h>

/**
 * defined required functions for adoubles
 */
inline adouble abs(const adouble& val) {
    return fabs(val);
}

inline adouble sign(const adouble& val) {
    throw CppAD::cg::CGException("Sign operation not implemented in ADOL-C");
    //adouble temp;
    //condassign(temp, val, adouble(1.0), adouble(-1.0)); // warning it should return zero if val == 0 but there is no way to do this in Adol-C
    //return temp;
}

inline adouble condExpLt(const adouble& left, const adouble& right,
                         const adouble& exp_if_true, const adouble& exp_if_false) {
    adouble temp;
    condassign(temp, right - left, exp_if_true, exp_if_false); // temp = (right - left>0)? exp_if_true: exp_if_false
    return temp;
}

inline adouble condExpLe(const adouble& left, const adouble& right, const adouble& exp_if_true, const adouble& exp_if_false) {
    throw CppAD::cg::CGException("Conditional operation (<=) not implemented in ADOL-C");
}

inline adouble condExpEq(const adouble& left, const adouble& right,
                         const adouble& exp_if_true, const adouble& exp_if_false) {
    throw CppAD::cg::CGException("Conditional operation (==) not implemented in ADOL-C");
}

inline adouble condExpGe(const adouble& left, const adouble& right,
                         const adouble& exp_if_true, const adouble& exp_if_false) {
    throw CppAD::cg::CGException("Conditional operation (>=) not implemented in ADOL-C");
}

inline adouble condExpGt(const adouble& left, const adouble& right,
                         const adouble& exp_if_true, const adouble& exp_if_false) {
    adouble temp;
    condassign(temp, left - right, exp_if_true, exp_if_false);
    return temp;
}

inline adouble condExpNe(const adouble& left, const adouble& right,
                         const adouble& exp_if_true, const adouble& exp_if_false) {
    throw CppAD::cg::CGException("Conditional operation (!=) not implemented in ADOL-C");
}

inline adouble CondExpOp(enum CppAD::CompareOp cop,
                         const adouble& left,
                         const adouble& right,
                         const adouble& trueCase,
                         const adouble& falseCase) {
    switch (cop) {
        case CppAD::CompareLt: // less than
            return condExpLt(left, right, trueCase, falseCase);
        case CppAD::CompareLe: // less than or equal
            return condExpLe(left, right, trueCase, falseCase);
        case CppAD::CompareEq: // equal
            return condExpEq(left, right, trueCase, falseCase);
        case CppAD::CompareGe: // greater than or equal
            return condExpGe(left, right, trueCase, falseCase);
        case CppAD::CompareGt: // greater than
            return condExpGt(left, right, trueCase, falseCase);
        case CppAD::CompareNe:
            return condExpNe(left, right, trueCase, falseCase);
        default:
            throw CppAD::cg::CGException("Unknown comparison type");
    }
}

namespace CppAD {
namespace cg {

/**
 * Specialization of class Evaluator for Adol-C
 */
template<class ScalarIn>
class Evaluator<ScalarIn, double, adouble> : public EvaluatorOperations<ScalarIn, double, adouble, Evaluator<ScalarIn, double, adouble> > {
public:
    using ActiveOut = adouble;
    using Super = EvaluatorOperations<ScalarIn, double, adouble, Evaluator<ScalarIn, double, adouble> >;
public:

    inline Evaluator(CodeHandler<ScalarIn>& handler) :
        Super(handler) {
    }

    inline virtual ~Evaluator() = default;

};

} // END cg namespace
} // END CppAD namespace

#endif
