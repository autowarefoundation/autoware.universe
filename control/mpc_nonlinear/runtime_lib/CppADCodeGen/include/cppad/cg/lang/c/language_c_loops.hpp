#ifndef CPPAD_CG_LANGUAGE_C_LOOPS_INCLUDED
#define CPPAD_CG_LANGUAGE_C_LOOPS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
 *    Copyright (C) 2019 Joao Leal
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
void LanguageC<Base>::pushLoopIndexedDep(OperationNode <Base>& node) {
    CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for loop indexed dependent operation")

    // LoopIndexedDep
    push(node.getArguments()[0]);
}

template<class Base>
size_t LanguageC<Base>::printLoopIndexDeps(const std::vector<OperationNode<Base>*>& variableOrder,
                                           size_t pos) {
    CPPADCG_ASSERT_KNOWN(pos < variableOrder.size(), "Invalid number of arguments for array creation operation")
    CPPADCG_ASSERT_KNOWN(variableOrder[pos]->getOperationType() == CGOpCode::LoopIndexedDep, "Invalid operation type")

    const size_t vSize = variableOrder.size();

    for (size_t i = pos; i < vSize; i++) {
        if (variableOrder[i]->getOperationType() != CGOpCode::LoopIndexedDep) {
            return i - 1;
        }

        // try to use a loop for element assignment
        size_t newI = printLoopIndexedDepsUsingLoop(variableOrder, i);

        if (newI == i) {
            // individual element assignment
            printAssignment(*variableOrder[i]);
        } else {
            i = newI;
        }
    }

    return vSize - 1;
}

template<class Base>
inline size_t LanguageC<Base>::printLoopIndexedDepsUsingLoop(const std::vector<OperationNode<Base>*>& variableOrder,
                                                             size_t starti) {
    CPPADCG_ASSERT_KNOWN(variableOrder[starti] != nullptr, "Invalid node")
    CPPADCG_ASSERT_KNOWN(variableOrder[starti]->getOperationType() == CGOpCode::LoopIndexedDep, "Invalid operation type")

    const size_t vSize = variableOrder.size();

    const OperationNode<Base>& ref = *variableOrder[starti];

    const IndexPattern* refIp = _info->loopDependentIndexPatterns[ref.getInfo()[0]];
    size_t refAssignOrAdd = ref.getInfo()[1];

    /**
     * Check that the assigned value is from an array
     */
    const OperationNode<Base>* refLeft = ref.getArguments()[0].getOperation();
    if (refLeft == nullptr) {
        return starti;
    } else if (refLeft->getOperationType() != CGOpCode::ArrayElement) {
        return starti;
    }

    /**
     * check that the type of index pattern can be used within a loop
     */
    const LinearIndexPattern* refLIp = nullptr;
    const SectionedIndexPattern* refSecp = nullptr;

    if (refIp->getType() == IndexPatternType::Linear) {
        refLIp = static_cast<const LinearIndexPattern*> (refIp);
    } else if (refIp->getType() == IndexPatternType::Sectioned) {
        refSecp = static_cast<const SectionedIndexPattern*> (refIp);
    } else {
        return starti; // cannot determine consecutive elements
    }

    /**
     * Find last compatible variable in variableOrder
     */
    const OperationNode<Base>* refArray = refLeft->getArguments()[0].getOperation();
    const size_t startArrayIndex = refLeft->getInfo()[0];

    size_t i = starti + 1;

    for (; i < vSize; i++) {
        OperationNode<Base>* node = variableOrder[i];
        if (node->getOperationType() != CGOpCode::LoopIndexedDep)
            break;

        const OperationNode<Base>* nodeLeft = variableOrder[i]->getArguments()[0].getOperation();
        if (nodeLeft->getOperationType() != CGOpCode::ArrayElement)
            break;

        const OperationNode<Base>* arrayi = nodeLeft->getArguments()[0].getOperation();
        if (arrayi != refArray)
            break;

        long offset = long(i) - long(starti);

        if (nodeLeft->getInfo()[0] != startArrayIndex + offset)
            break;

        if (node->getInfo()[1] != refAssignOrAdd)
            break;

        const IndexPattern* ip = _info->loopDependentIndexPatterns[node->getInfo()[0]];
        if (!isOffsetBy(ip, refIp, offset)) {
            break; // different pattern type
        }
    }

    if (i - starti < 3)
        return starti; // no point in looping for 2 elements

    /**
     * Create the dependent variable name with the new index pattern
     */
    std::unique_ptr<Plane2DIndexPattern> p2dip;
    if (refLIp != nullptr) {
        p2dip.reset(encapsulateIndexPattern(*refLIp, 0));
    } else {
        assert(refSecp != nullptr);
        p2dip.reset(encapsulateIndexPattern(*refSecp, 0));
    }

    std::unique_ptr<OperationNode<Base>> op2(OperationNode<Base>::makeTemporaryNode(CGOpCode::LoopIndexedDep, ref.getInfo(), ref.getArguments()));
    op2->getInfo()[1] = (std::numeric_limits<size_t>::max)(); // just to be safe (this would be the index pattern id in the handler)
    op2->getArguments().push_back(_info->auxIterationIndexOp);

    std::ostringstream rightAssign;
    
    rightAssign << _nameGen->generateIndexedDependent(*op2, 0, *p2dip);

    /**
     * print the loop
     */
    size_t depVarCount = i - starti;
    _streamStack << _indentation << "for(i = 0; i < " << depVarCount << "; i++) ";
    _streamStack << rightAssign.str() << " ";
    if (refAssignOrAdd == 1) {
        _streamStack << "+=";
    } else {
        _streamStack << _depAssignOperation;
    }
    _streamStack << " ";

    std::string arrayName;
    if (refArray->getOperationType() == CGOpCode::ArrayCreation)
        arrayName = _nameGen->generateTemporaryArray(*refArray, getVariableID(*refArray));
    else
        arrayName = _nameGen->generateTemporarySparseArray(*refArray, getVariableID(*refArray));

    _streamStack << "(" << arrayName << ")[i + " << startArrayIndex << "];\n";

    return i - 1;
}


} // END cg namespace
} // END CppAD namespace

#endif