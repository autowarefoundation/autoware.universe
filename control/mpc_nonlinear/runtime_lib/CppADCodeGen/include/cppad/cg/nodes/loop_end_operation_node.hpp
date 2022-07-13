#ifndef CPPAD_CG_LOOP_END_OPERATION_NODE_INCLUDED
#define CPPAD_CG_LOOP_END_OPERATION_NODE_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
 *    Copyright (C) 2020 Joao Leal
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
 * An operation node that marks the end of a loop.
 * 
 * This is a custom OperationNode class and therefore cannot be transformed
 * into any other node type (makeAlias() and setOperation() might not work).
 * 
 * @author Joao Leal
 */
template<class Base>
class LoopEndOperationNode : public OperationNode<Base> {
    friend class CodeHandler<Base>;
public:

    inline const LoopStartOperationNode<Base>& getLoopStart() const {
        const std::vector<Argument<Base> >& args = this->getArguments();
        CPPADCG_ASSERT_KNOWN(args.size() > 0, "There must be at least one argument")

        OperationNode<Base>* aNode = args[0].getOperation();
        CPPADCG_ASSERT_KNOWN(aNode != nullptr && aNode->getOperationType() == CGOpCode::LoopStart, "The first argument must be the loop start operation")

        return dynamic_cast<LoopStartOperationNode<Base>&> (*aNode);
    }

    inline virtual ~LoopEndOperationNode() = default;

protected:

    inline LoopEndOperationNode(CodeHandler<Base>* handler,
                                LoopStartOperationNode<Base>& loopStart,
                                const std::vector<Argument<Base> >& endArgs) :
        OperationNode<Base>(handler, CGOpCode::LoopEnd, std::vector<size_t>(0), createArguments(loopStart, endArgs)) {
    }

private:

    static inline std::vector<Argument<Base> > createArguments(LoopStartOperationNode<Base>& lstart,
                                                               const std::vector<Argument<Base> >& endArgs) {
        std::vector<Argument<Base> > args(1 + endArgs.size());
        args[0] = lstart;
        std::copy(endArgs.begin(), endArgs.end(), args.begin() + 1);
        return args;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif