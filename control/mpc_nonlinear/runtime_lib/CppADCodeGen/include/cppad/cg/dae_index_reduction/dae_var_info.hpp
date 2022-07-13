#ifndef CPPAD_CG_DAE_VAR_INFO_INCLUDED
#define CPPAD_CG_DAE_VAR_INFO_INCLUDED
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
 * DAE variable information
 */
class DaeVarInfo {
private:
    /**
     * A unique identifier for this variable (used internally)
     */
    size_t id_;
    /**
     * The index of the variable for which this variable is the time
     * derivative. A negative value means that the current variable isn't 
     * a time derivative.
     */
    int antiDerivative_;
    /**
     * The index of the time derivative of this variable. A negative value
     * means that there is none.
     */
    int derivative_;
    /**
     *  Defines the independent variables that dependent on the integrated
     *  variable
     */
    bool integratedDependent_;
    /**
     * Whether or not this variable is an integrated variable (usually 
     * associated with time)
     */
    bool integratedVariable_;
    /**
     * A custom variable name (keep it empty to use an automatically 
     * generated name)
     */
    std::string name_;
    /**
     * The variable tape index in the original model
     */
    int originalIndex_;
    /**
     * Time derivative order. A negative value means that it is a constant.
     * An order higher than zero does not mean that the variable should be
     * treated as a differential variable.
     */
    int order_;
    /**
     * The original variable index for which this variable is the 
     * time derivative (with the order provided by order_). A negative value
     * means that the current variable was never a time derivative.
     */
    int originalAntiDerivative_;
public:

    /**
     * Creates a new DAE variable
     * 
     * @param name A custom variable name (keep it empty to use an
     *             automatically generated name)
     * @param id A unique identifier for this variable (used internally)
     */
    inline DaeVarInfo(const std::string& name = "",
                      size_t id = 0) :
        id_(id),
        antiDerivative_(-1),
        derivative_(-1),
        integratedDependent_(true),
        integratedVariable_(false),
        name_(name),
        originalIndex_(-1),
        order_(0),
        originalAntiDerivative_(-1) {
    }

    inline DaeVarInfo(int derivativeOf,
                      const std::string& name = "",
                      size_t id = 0) :
        id_(id),
        antiDerivative_(derivativeOf),
        derivative_(-1),
        integratedDependent_(true),
        integratedVariable_(false),
        name_(name),
        originalIndex_(-1),
        order_(0),
        originalAntiDerivative_(-1) {
    }

    /**
     * Provides a unique identifier for the variable.
     * 
     * @return a unique identifier for the variable.
     */
    inline size_t getId() const {
        return id_;
    }

    inline void setId(size_t id) {
        id_ = id;
    }

    /**
     * The index of the variable that the current variable is the derivative
     * of. A negative value means that the current variable isn't a 
     * derivative.
     * 
     * @return The index of the variable for which this variable is the 
     *         derivative of.
     */
    inline int getAntiDerivative() const {
        return antiDerivative_;
    }

    inline void setAntiDerivative(int derivativeOf) {
        antiDerivative_ = derivativeOf;
    }

    /**
     * The index of the time derivative for this variable. A negative value
     * means that there is none.
     * 
     * @return The index of the time derivative for this variable.
     */
    inline int getDerivative() const {
        return derivative_;
    }

    /**
     * Defines the index of the time derivative for this variable. 
     * 
     * @param derivative The index of the time derivative for this variable.
     *                   A negative value means that there is none.
     */
    inline void setDerivative(int derivative) {
        derivative_ = derivative;
    }

    /**
     * Determines whether or not this variable depends on the 
     * independent/integrated variables.
     * 
     * @return true if it is a parameter that does not depend on the
     *              integrated variables
     */
    inline bool isFunctionOfIntegrated() const {
        return integratedDependent_;
    }

    /**
     * Defines this variable as a parameter/constant that does not depend on
     * the independent/integrated variables
     */
    inline void makeConstant() {
        integratedVariable_ = false;
        integratedDependent_ = false;
        antiDerivative_ = -1;
        order_ = -1;
        originalAntiDerivative_ = -1;
    }

    /**
     * Defines this variable as an integrated variable, also known
     * as the independent variable of the DAE system (usually associated
     * with time)
     */
    inline void makeIntegratedVariable() {
        integratedVariable_ = true;
        integratedDependent_ = false;
        antiDerivative_ = -1;
        order_ = -1;
        originalAntiDerivative_ = -1;
    }

    /**
     * Determines whether or not this is an integrated variable, also known
     * as the independent variable of the DAE system (typically time).
     * 
     * @return true if it is the integrated variable
     */
    inline bool isIntegratedVariable() const {
        return integratedVariable_;
    }

    /**
     * Returns the custom variable name. If the string is empty an 
     * automatically generated name will be used.
     * 
     * @return the custom variable name
     */
    inline const std::string& getName() const {
        return name_;
    }

    /**
     * Defines a custom variable name. If the string is empty an 
     * automatically generated name will be used.
     * 
     * @param name the custom variable name
     */
    inline void setName(const std::string& name) {
        name_ = name;
    }

    /**
     * Provides the variable index corresponding to the original model.
     * 
     * @return The corresponding variable index in the original model. A 
     *         negative value means that this variable was created by the
     *         algorithm.
     */
    inline int getOriginalIndex() const {
        return originalIndex_;
    }

    /**
     * Defines the variable index in the original model.
     * 
     * @param originalIndex The corresponding variable index in the original
     *                      model. A negative value means that this variable
     *                      was created by the algorithm.
     */
    inline void setOriginalIndex(int originalIndex) {
        originalIndex_ = originalIndex;
    }

    /**
     * Provides the original variable index for which this variable is/was
     * the time derivative (with the order provided by order_).
     * A negative value means that the current variable was never a time 
     * derivative.
     * A non-negative value does not mean that this variable should be 
     * treated as a time derivative since it might have been transformed 
     * into an algebraic variable by the algorithm.
     * 
     * @return the index in the original model for which this variable is 
     *         the time derivative
     */
    inline int getOriginalAntiDerivative() const {
        return originalAntiDerivative_;
    }

    /**
     * Defines the original variable index for which this variable is/was
     * the time derivative (with the order provided by order_).
     * A negative value means that the current variable was never a time 
     * derivative.
     * A non-negative value does not mean that this variable should be 
     * treated as a time derivative since it might have been transformed 
     * into an algebraic variable by the algorithm.
     * 
     * @param originalAntiDerivative the index in the original model for
     *                               which this variable is the time
     *                               derivative
     */
    inline void setOriginalAntiDerivative(int originalAntiDerivative) {
        originalAntiDerivative_ = originalAntiDerivative;
    }

    /**
     * Provides the order of a time derivative.
     * A negative value means that it is a constant.
     * An order higher than zero does not mean that the variable should be
     * treated as a time derivative, it could very well be a time derivative 
     * transformed into an algebraic variable by the algorithm.
     *
     * @return The order of the time derivative
     */
    inline int getOrder() const {
        return order_;
    }

    /**
     * Defines the order of a time derivative.
     * A negative value means that it is a constant.
     * An order higher than zero does not mean that the variable should be
     * treated as a time derivative, it could very well be a time derivative 
     * transformed into an algebraic variable by the algorithm.
     * 
     * @param order The order of the time derivative
     */
    inline void setOrder(int order) {
        order_ = order;
    }

    inline void printInfo(std::ostream& out = std::cout) const {
        out << name_ << ":\n";
        if (antiDerivative_ >= 0)
            out << " derivative-of: " << antiDerivative_ << "\n";
        if (derivative_ >= 0)
            out << " derivative: " << derivative_ << "\n";
        if (integratedDependent_)
            out << " integrated dependent\n";
        else if (integratedVariable_)
            out << " integrated variable\n";
        out.flush();
    }

    inline virtual ~DaeVarInfo() {
    }
};

} // END cg namespace
} // END CppAD namespace

#endif