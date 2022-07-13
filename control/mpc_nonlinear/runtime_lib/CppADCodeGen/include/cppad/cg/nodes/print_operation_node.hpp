#ifndef CPPAD_CG_PRINT_OPERATION_NODE_INCLUDED
#define CPPAD_CG_PRINT_OPERATION_NODE_INCLUDED
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
 * An operation node which prints out a variable value or parameter.
 * 
 * This is a custom OperationNode class and therefore cannot be transformed
 * into any other node type (makeAlias() and setOperation() might not work).
 * 
 * @author Joao Leal
 */
template<class Base>
class PrintOperationNode : public OperationNode<Base> {
    friend class CodeHandler<Base>;
protected:
    std::string before_;
    std::string after_;
public:

    inline const std::string& getBeforeString() const {
        return before_;
    }

    inline void setBeforeString(const std::string& before) {
        before_ = before;
    }

    inline const std::string& getAfterString() const {
        return after_;
    }

    inline void setAfterString(const std::string& after) {
        after_ = after;
    }

    inline virtual ~PrintOperationNode() {
    }

protected:

    inline PrintOperationNode(CodeHandler<Base>* handler,
                              const std::string& before,
                              const Argument<Base>& arg,
                              const std::string& after) :
        OperationNode<Base>(handler, CGOpCode::Pri, arg),
        before_(before),
        after_(after) {
    }

};

} // END cg namespace
} // END CppAD namespace

#endif