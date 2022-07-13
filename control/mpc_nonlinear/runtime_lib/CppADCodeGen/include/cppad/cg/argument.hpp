#ifndef CPPAD_CG_ARGUMENT_INCLUDED
#define CPPAD_CG_ARGUMENT_INCLUDED
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
class OperationNode;

/**
 * An argument used by an operation which can be either a constant value
 * or the result of another operation
 *
 * @author Joao Leal
 */
template<class Base>
class Argument {
private:
    OperationNode<Base>* operation_;
    std::unique_ptr<Base> parameter_;
public:

    inline Argument() :
        operation_(nullptr) {
    }

    inline Argument(OperationNode<Base>& operation) :
        operation_(&operation) {
    }

    inline Argument(const Base& parameter) :
        operation_(nullptr),
        parameter_(new Base(parameter)) {
    }

    inline Argument(const Argument& orig) :
        operation_(orig.operation_),
        parameter_(orig.parameter_ != nullptr ? new Base(*orig.parameter_) : nullptr) {
    }

    inline Argument(Argument&& orig) :
            operation_(orig.operation_),
            parameter_(std::move(orig.parameter_)) {
    }

    inline Argument& operator=(const Argument& rhs) {
        if (&rhs == this) {
            return *this;
        }
        if (rhs.operation_ != nullptr) {
            operation_ = rhs.operation_;
            parameter_.reset();
        } else {
            operation_ = nullptr;
            if (parameter_ != nullptr) {
                *parameter_ = *rhs.parameter_;
            } else {
                parameter_.reset(new Base(*rhs.parameter_)); // to replace with parameter_ = std::make_unique once c++14 is used
            }
        }
        return *this;
    }

    inline Argument& operator=(Argument&& rhs) {
        assert(this != &rhs);

        operation_ = rhs.operation_;

        // steal the parameter
        parameter_ = std::move(rhs.parameter_);

        return *this;
    }

    virtual ~Argument() = default;

    inline OperationNode<Base>* getOperation() const {
        return operation_;
    }

    inline Base* getParameter() const {
        return parameter_.get();
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
