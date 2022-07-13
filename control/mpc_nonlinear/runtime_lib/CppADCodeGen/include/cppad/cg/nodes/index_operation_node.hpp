#ifndef CPPAD_CG_INDEX_OPERATION_NODE_INCLUDED
#define CPPAD_CG_INDEX_OPERATION_NODE_INCLUDED
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
 * An index reference operation node
 * 
 * This is a custom OperationNode class and therefore cannot be transformed
 * into any other node type (makeAlias() and setOperation() might not work).
 * 
 * @author Joao Leal
 */
template<class Base>
class IndexOperationNode : public OperationNode<Base> {
    friend class CodeHandler<Base>;
public:

    inline bool isDefinedLocally() const {
        return this->getArguments().size() > 1;
    }

    inline OperationNode<Base>& getIndexCreationNode() const {
        const std::vector<Argument<Base> >& args = this->getArguments();
        CPPADCG_ASSERT_KNOWN(!args.empty(), "Invalid number of arguments");
        CPPADCG_ASSERT_KNOWN(args.back().getOperation() != nullptr, "Invalid argument type");
        return *args.back().getOperation();
    }

    inline const OperationNode<Base>& getIndex() const {
        const std::vector<Argument<Base> >& args = this->getArguments();
        CPPADCG_ASSERT_KNOWN(!args.empty(), "Invalid number of arguments");

        OperationNode<Base>* aNode = args[0].getOperation();
        CPPADCG_ASSERT_KNOWN(aNode != nullptr && aNode->getOperationType() == CGOpCode::IndexDeclaration, "Invalid argument operation type");

        return static_cast<const OperationNode<Base>&> (*aNode);
    }

    inline void makeAssigmentDependent(IndexAssignOperationNode<Base>& indexAssign) {
        std::vector<Argument<Base> >& args = this->getArguments();

        args.resize(2);
        args[0] = indexAssign.getIndex();
        args[1] = indexAssign;
    }

    inline virtual ~IndexOperationNode() {
    }

protected:

    inline IndexOperationNode(CodeHandler<Base>* handler,
                              OperationNode<Base>& indexDcl) :
        OperationNode<Base>(handler, CGOpCode::Index, indexDcl) {
    }

    inline IndexOperationNode(CodeHandler<Base>* handler,
                              LoopStartOperationNode<Base>& loopStart) :
        OperationNode<Base>(handler, CGOpCode::Index,{loopStart.getIndex(), loopStart}) {
    }

    inline IndexOperationNode(CodeHandler<Base>* handler,
                              IndexAssignOperationNode<Base>& indexAssign) :
        OperationNode<Base>(handler, CGOpCode::Index,{indexAssign.getIndex(), indexAssign}) {
    }

};

} // END cg namespace
} // END CppAD namespace

#endif