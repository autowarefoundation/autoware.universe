#ifndef CPPAD_CG_DAE_EQUATION_INFO_HPP
#define	CPPAD_CG_DAE_EQUATION_INFO_HPP
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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
 * DAE equation information
 */
class DaeEquationInfo {
private:
    /**
     * A unique identifier for this equation (used internally)
     */
    size_t id_;
    /**
     * The equation index in the original user model
     */
    int originalIndex_;
    /**
     * The index of the equation that was differentiated to obtain this
     * equation. A negative value means that the current equation isn't a
     * differentiation of an existing equation.
     */
    int antiDerivative_;
    /**
     * The variable index associated with this equation. A negative value is
     * used if this equation does not have an assigned variable.
     */
    int assignedVarIndex_;
    /**
     * Whether or not if it is an explicit differential equation
     */
    bool explicit_;
public:

    inline DaeEquationInfo() :
        id_(0),
        originalIndex_(-1),
        antiDerivative_(-1),
        assignedVarIndex_(-1),
        explicit_(false) {
    }

    inline DaeEquationInfo(size_t id,
                           int originalIndex,
                           int derivativeOf,
                           int assignedVarIndex,
                           bool explicitEq = false) :
        id_(id),
        originalIndex_(originalIndex),
        antiDerivative_(derivativeOf),
        assignedVarIndex_(assignedVarIndex),
        explicit_(explicitEq) {
    }

    /**
     * Provides a unique identifier for the equation.
     * 
     * @return a unique identifier for the equation.
     */
    inline size_t getId() const {
        return id_;
    }

    inline void setId(size_t id) {
        id_ = id;
    }

    inline int getAntiDerivative() const {
        return antiDerivative_;
    }

    inline void setAntiDerivative(int derivativeOf) {
        antiDerivative_ = derivativeOf;
    }

    inline int getAssignedVarIndex() const {
        return assignedVarIndex_;
    }

    inline void setAssignedVarIndex(int assignedVarIndex) {
        assignedVarIndex_ = assignedVarIndex;
    }

    inline int getOriginalIndex() const {
        return originalIndex_;
    }

    inline bool isExplicit() const {
        return explicit_;
    }

    inline void setExplicit(bool explicitEq) {
        explicit_ = explicitEq;
    }

    inline virtual ~DaeEquationInfo() {
    }
};

} // END cg namespace
} // END CppAD namespace

#endif