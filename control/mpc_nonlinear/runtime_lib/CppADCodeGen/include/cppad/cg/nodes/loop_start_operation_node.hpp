#ifndef CPPAD_CG_LOOP_START_OPERATION_NODE_INCLUDED
#define CPPAD_CG_LOOP_START_OPERATION_NODE_INCLUDED
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
 * An operation node that marks the beginning of a loop.
 * 
 * This is a custom OperationNode class and therefore cannot be transformed
 * into any other node type (makeAlias() and setOperation() might not work).
 * 
 * @author Joao Leal
 */
template<class Base>
class LoopStartOperationNode : public OperationNode<Base> {
        friend class CodeHandler<Base>;
public:

    inline OperationNode<Base>& getIndex() const {
        const std::vector<Argument<Base> >& args = this->getArguments();
        CPPADCG_ASSERT_KNOWN(!args.empty(), "Invalid number of arguments")

        OperationNode<Base>* aNode = args[0].getOperation();
        CPPADCG_ASSERT_KNOWN(aNode != nullptr && aNode->getOperationType() == CGOpCode::IndexDeclaration, "Invalid argument operation type")

        return static_cast<OperationNode<Base>&> (*aNode);
    }

    inline IndexOperationNode<Base>* getIterationCountNode() const {
        if (this->getInfo().empty()) {
            CPPADCG_ASSERT_KNOWN(this->getArguments().size() > 1, "Invalid number of arguments.")

            OperationNode<Base>* aNode = this->getArguments()[1].getOperation();
            CPPADCG_ASSERT_KNOWN(aNode != nullptr && aNode->getOperationType() == CGOpCode::Index, "Invalid argument node type")

            return static_cast<IndexOperationNode<Base>*> (aNode);
        }

        return nullptr;
    }

    inline const size_t getIterationCount() const {
        if (this->getInfo().empty()) {
            return 0;
        }
        return this->getInfo()[0];
    }

    inline virtual ~LoopStartOperationNode() = default;

protected:

    inline LoopStartOperationNode(CodeHandler<Base>* handler,
                                  OperationNode<Base>& indexDcl,
                                  size_t iterationCount) :
        OperationNode<Base>(handler, CGOpCode::LoopStart, indexDcl) {
        this->getInfo().push_back(iterationCount);
    }

    inline LoopStartOperationNode(CodeHandler<Base>* handler,
                                  OperationNode<Base>& indexDcl,
                                  IndexOperationNode<Base>& iterCount) :
        OperationNode<Base>(handler, CGOpCode::LoopStart,{indexDcl, iterCount}) {
    }

};

} // END cg namespace
} // END CppAD namespace

#endif