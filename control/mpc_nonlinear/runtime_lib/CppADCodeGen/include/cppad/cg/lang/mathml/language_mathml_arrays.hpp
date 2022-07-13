#ifndef CPPAD_CG_LANGUAGE_MATHML_ARRAYS_INCLUDED
#define CPPAD_CG_LANGUAGE_MATHML_ARRAYS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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
void LanguageMathML<Base>::printArrayCreationOp(OperationNode<Base>& array) {
    CPPADCG_ASSERT_KNOWN(array.getArguments().size() > 0, "Invalid number of arguments for array creation operation")
    const size_t id = getVariableID(array);
    const std::vector<Argument<Base> >& args = array.getArguments();
    const size_t argSize = args.size();

    size_t startPos = id - 1;

    bool firstElement = true;
    for (size_t i = 0; i < argSize; i++) {
        bool newValue = !isSameArgument(args[i], _tmpArrayValues[startPos + i]);

        if (newValue) {
            if (firstElement) {
                _code << _startEq
                        << "<mrow class='tmp'>" << auxArrayName_ << "</mrow>" << _assignStr << _nameGen->generateTemporaryArray(array, getVariableID(array))
                        << _endEq
                        << " <!-- size: " << args.size() << " -->" << _endline;
                firstElement = false;
            }

            // try to use a loop for element assignment
            size_t newI = printArrayCreationUsingLoop(startPos, array, i, _tmpArrayValues);

            // print elements not assign in a loop
            if (newI == i) {
                // individual element assignment
                _code << _startEq
                        << "<mrow class='tmp'>" << auxArrayName_ << "</mrow><mfenced open='[' close=']'><mn>" << i << "</mn></mfenced>" << _assignStr;
                print(args[i]);
                _code << _endEq << _endline;

                _tmpArrayValues[startPos + i] = &args[i];

            } else {
                i = newI - 1;
            }
        }
    }
}

template<class Base>
void LanguageMathML<Base>::printSparseArrayCreationOp(OperationNode<Base>& array) {
    const std::vector<size_t>& info = array.getInfo();
    CPPADCG_ASSERT_KNOWN(!info.empty(), "Invalid number of information elements for sparse array creation operation")

    const std::vector<Argument<Base> >& args = array.getArguments();
    const size_t argSize = args.size();

    CPPADCG_ASSERT_KNOWN(info.size() == argSize + 1, "Invalid number of arguments for sparse array creation operation")

    if (argSize == 0)
        return; // empty array

    const size_t id = getVariableID(array);
    size_t startPos = id - 1;

    bool firstElement = true;
    for (size_t i = 0; i < argSize; i++) {
        bool newValue = !isSameArgument(args[i], _tmpSparseArrayValues[startPos + i]);

        if (newValue) {
            if (firstElement) {
                _code << _startEq
                        << "<mrow class='tmp'>" << auxArrayName_ << "</mrow>" << _assignStr << _nameGen->generateTemporarySparseArray(array, getVariableID(array))
                        << _endEq
                        << " <!-- nnz: " << args.size() << "  size:" << info[0] << " -->" << _endline;
                firstElement = false;
            }

            // try to use a loop for element assignment
            size_t newI = printArrayCreationUsingLoop(startPos, array, i, _tmpSparseArrayValues);

            // print element values not assign in a loop
            if (newI == i) {
                // individual element assignment
                _code << _startEq
                        << "<mrow class='tmp'>" << auxArrayName_ << "</mrow><mfenced open='[' close=']'><mn>" << i << "</mn></mfenced>" << _assignStr;
                print(args[i]);
                _code << _endEq;
                _code << ", ";
                // print indexes (location of values)
                _code << _startEq
                        << "<mi>" << _C_SPARSE_INDEX_ARRAY << "</mi><mfenced open='[' close=']'><mrow>";
                if (startPos != 0) _code << "<mn>" << startPos << "</mn><mo>+</mo>";
                _code << "<mn>" << i << "</mn></mrow></mfenced>" << _assignStr << "<mn>" << info[i + 1] << "</mn>"
                        << _endEq << _endline;

                _tmpSparseArrayValues[startPos + i] = &args[i];

            } else {
                // print indexes (location of values)
                for (size_t j = i; j < newI; j++) {
                    _code << _startEq
                            << "<mi>" << _C_SPARSE_INDEX_ARRAY << "</mi><mfenced open='[' close=']'><mrow>";
                    if (startPos != 0) _code << "<mn>" << startPos << "</mn><mo>+</mo>";
                    _code << "<mn>" << j << "</mn></mrow></mfenced>" << _assignStr << "<mn>" << info[j + 1] << "</mn>"
                            << _endEq << _endline;
                }

                i = newI - 1;
            }


        } else {
            // print indexes (location of values)
            _code << _startEq
                    << "<mi>" << _C_SPARSE_INDEX_ARRAY << "</mi><mfenced open='[' close=']'><mrow>";
            if (startPos != 0) _code << "<mn>" << startPos << "</mn><mo>+</mo>";
            _code << "<mn>" << i << "</mn></mrow></mfenced>" << _assignStr << "<mn>" << info[i + 1] << "</mn>"
                    << _endEq << _endline;
        }

    }
}

template<class Base>
inline size_t LanguageMathML<Base>::printArrayCreationUsingLoop(size_t startPos,
                                                                OperationNode<Base>& array,
                                                                size_t starti,
                                                                std::vector<const Argument<Base>*>& tmpArrayValues) {
    const std::vector<Argument<Base> >& args = array.getArguments();
    const size_t argSize = args.size();
    size_t i = starti + 1;

    std::ostringstream arrayAssign;

    const Argument<Base>& ref = args[starti];
    if (ref.getOperation() != nullptr) {
        // 
        const OperationNode<Base>& refOp = *ref.getOperation();
        if (refOp.getOperationType() == CGOpCode::Inv) {
            /**
             * from independents array
             */
            for (; i < argSize; i++) {
                if (isSameArgument(args[i], tmpArrayValues[startPos + i]))
                    break; // no assignment needed

                if (args[i].getOperation() == nullptr ||
                        args[i].getOperation()->getOperationType() != CGOpCode::Inv ||
                        !_nameGen->isConsecutiveInIndepArray(*args[i - 1].getOperation(), getVariableID(*args[i - 1].getOperation()),
                                                             *args[i].getOperation(), getVariableID(*args[i].getOperation()))) {
                    break;
                }
            }

            if (i - starti < 3)
                return starti;

            // use loop
            const std::string& indep = _nameGen->getIndependentArrayName(refOp, getVariableID(refOp));
            size_t start = _nameGen->getIndependentArrayIndex(refOp, getVariableID(refOp));
            long offset = long(start) - starti;
            if (offset == 0)
                arrayAssign << "<mrow class='indep'>" << indep << "</mrow>" << "<mfenced open='[' close=']'><mi>i</mi></mfenced>";
            else
                arrayAssign << "<mrow class='indep'>" << indep << "</mrow>" << "<mfenced open='[' close=']'><mrow><mn>" << offset << "</mn> <mo>+</mo> <mi>i</mi></mrow></mfenced>";

        } else if (refOp.getOperationType() == CGOpCode::LoopIndexedIndep) {
            /**
             * from independents array in a loop
             */
            size_t pos = refOp.getInfo()[1];
            IndexPattern* refIp = _info->loopIndependentIndexPatterns[pos];
            if (refIp->getType() != IndexPatternType::Linear) {
                return starti; // cannot determine consecutive elements
            }

            auto* refLIp = static_cast<LinearIndexPattern*> (refIp);

            for (; i < argSize; i++) {
                if (isSameArgument(args[i], tmpArrayValues[startPos + i]))
                    break; // no assignment needed

                if (args[i].getOperation() == nullptr ||
                        args[i].getOperation()->getOperationType() != CGOpCode::LoopIndexedIndep) {
                    break; // not an independent index pattern
                }

                if (!_nameGen->isInSameIndependentArray(refOp, getVariableID(refOp),
                                                        *args[i].getOperation(), getVariableID(*args[i].getOperation())))
                    break;

                pos = args[i].getOperation()->getInfo()[1];
                const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
                if (ip->getType() != IndexPatternType::Linear) {
                    break; // different pattern type
                }
                const auto* lIp = static_cast<const LinearIndexPattern*> (ip);
                if (refLIp->getLinearSlopeDx() != lIp->getLinearSlopeDx() ||
                        refLIp->getLinearSlopeDy() != lIp->getLinearSlopeDy() ||
                        refLIp->getXOffset() != lIp->getXOffset() ||
                        refLIp->getLinearConstantTerm() - long(starti) != lIp->getLinearConstantTerm() - long(i)) {
                    break;
                }
            }

            if (i - starti < 3)
                return starti;

            LinearIndexPattern* lip2 = new LinearIndexPattern(*refLIp);
            lip2->setLinearConstantTerm(lip2->getLinearConstantTerm() - starti);
            Plane2DIndexPattern p2dip(lip2,
                                      new LinearIndexPattern(0, 1, 1, 0));

            std::unique_ptr<OperationNode<Base>> op2(OperationNode<Base>::makeTemporaryNode(CGOpCode::LoopIndexedIndep, refOp.getInfo(), refOp.getArguments()));
            op2->getInfo()[1] = (std::numeric_limits<size_t>::max)(); // just to be safe (this would be the index pattern id in the handler)
            op2->getArguments().push_back(_info->auxIterationIndexOp);

            arrayAssign << _nameGen->generateIndexedIndependent(*op2, 0, p2dip);

        } else {
            // no loop used
            return starti;
        }
    } else {
        /**
         * constant value?
         */
        const Base& value = *args[starti].getParameter();
        for (; i < argSize; i++) {
            if (args[i].getParameter() == nullptr || *args[i].getParameter() != value) {
                break; // not the same constant value
            }

            const Argument<Base>* oldArg = tmpArrayValues[startPos + i];
            if (oldArg != nullptr && oldArg->getParameter() != nullptr && *oldArg->getParameter() == value) {
                break; // values are the same (no need to redefine)
            }
        }

        if (i - starti < 3)
            return starti;

        arrayAssign << value;
    }

    /**
     * print the loop
     */
    _code << _forStart << _startEq << "<mi>for</mi>"
            "<mfenced><mrow><mi>i</mi><mo>&isin;</mo>"
            "<mfenced open='[' close=']' separators=';'>"
            "<mn>" << starti << "</mn><mn>" << i << "</mn>"
            "</mfenced>"
            "</mrow></mfenced>" << _endEq << _endline
            << _forBodyStart;
    _indentationLevel++;
    _code << _startEq
            << "<mrow class='tmp'>" << auxArrayName_ << "</mrow><mfenced open='[' close=']'><mi>i</mi></mfenced>" << _assignStr << arrayAssign.str()
            << _endEq << _endline;
    _indentationLevel--;
    _code << _forBodyEnd << _endline << _forEnd << _endline;

    /**
     * update values in the global temporary array
     */
    for (size_t ii = starti; ii < i; ii++) {
        tmpArrayValues[startPos + ii] = &args[ii];
    }

    return i;
}

template<class Base>
inline std::string LanguageMathML<Base>::getTempArrayName(const OperationNode<Base>& op) {
    if (op.getOperationType() == CGOpCode::ArrayCreation)
        return _nameGen->generateTemporaryArray(op);
    else
        return _nameGen->generateTemporarySparseArray(op);
}

template<class Base>
void LanguageMathML<Base>::printArrayElementOp(OperationNode<Base>& op) {
    CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for array element operation")
    CPPADCG_ASSERT_KNOWN(op.getArguments()[0].getOperation() != nullptr, "Invalid argument for array element operation")
    CPPADCG_ASSERT_KNOWN(op.getInfo().size() == 1, "Invalid number of information indexes for array element operation")

    OperationNode<Base>& arrayOp = *op.getArguments()[0].getOperation();
    std::string arrayName;
    if (arrayOp.getOperationType() == CGOpCode::ArrayCreation)
        arrayName = _nameGen->generateTemporaryArray(arrayOp, getVariableID(arrayOp));
    else
        arrayName = _nameGen->generateTemporarySparseArray(arrayOp, getVariableID(arrayOp));

    _code << "<mo>(</mo><mrow class='tmp'>" << arrayName << "</mrow><mo>)</mo><mfenced open='[' close=']'><mn>" << op.getInfo()[0] << "</mn></mfenced>";
}

template<class Base>
inline void LanguageMathML<Base>::printArrayStructInit(const std::string& dataArrayName,
                                                       size_t pos,
                                                       const std::vector<OperationNode<Base>*>& arrays,
                                                       size_t k) {
    _ss.str("");
    _ss << "<mrow class='tmp'>" << dataArrayName << "</mrow><mfenced open='[' close=']'><mn>" << pos << "</mn></mfenced>";
    printArrayStructInit(_ss.str(), *arrays[k]);
}

template<class Base>
inline void LanguageMathML<Base>::printArrayStructInit(const std::string& dataArrayName,
                                                       OperationNode<Base>& array) {
    /**
     * TODO: finish this
     */
    const std::string& aName = createVariableName(array);

    if (array.getOperationType() == CGOpCode::ArrayCreation) {
        // dense array
        size_t size = array.getArguments().size();
        if (size > 0)
            _code << dataArrayName << "<mo>.</mo><mi>data</mi><mo>=</mo>" << aName;
        else
            _code << dataArrayName << "<mo>.</mo><mi>data</mi><mo>=</mo>NULL";
        _code << dataArrayName << "<mo>.</mo><mi>size</mi><mo>=</mo><mn>" << size << "</mn>"
                << dataArrayName << "<mo>.</mo><mi>sparse</mi><mo>=</mo><mn>" << false << "</mn>";
    } else {
        // sparse array
        CPPADCG_ASSERT_KNOWN(array.getOperationType() == CGOpCode::SparseArrayCreation, "Invalid node type")
        size_t nnz = array.getArguments().size();
        if (nnz > 0)
            _code << dataArrayName << "<mo>.</mo><mi>data</mi><mo>=</mo>" << aName;
        else
            _code << dataArrayName << "<mo>.</mo><mi>data</mi><mo>=</mo>NULL";
        _code << dataArrayName << "<mo>.</mo><mi>size</mi><mo>=</mo><mn>" << array.getInfo()[0] << "</mn>"
                << dataArrayName << "<mo>.</mo><mi>sparse</mi><mo>=</mo><mn>" << true << "</mn>"
                << dataArrayName << "<mo>.</mo><mi>nnz</mi><mo>=</mo><mn>" << nnz << "</mn>";
        if (nnz > 0) {
            size_t id = getVariableID(array);
            _code << dataArrayName << "<mo>.</mo><mi>idx</mi><mo>=</mo>&amp;<mo>(</mo><mi>" << _C_SPARSE_INDEX_ARRAY << "</mi><mfenced open='[' close=']'><mn>" << (id - 1) << "</mn></mfenced><mo>)</mo>";
        }
    }
    _code << _endline;
}

template<class Base>
inline void LanguageMathML<Base>::markArrayChanged(OperationNode<Base>& ty) {
    size_t id = getVariableID(ty);
    size_t tySize = ty.getArguments().size();

    if (ty.getOperationType() == CGOpCode::ArrayCreation) {
        for (size_t i = 0; i < tySize; i++) {
            _tmpArrayValues[id - 1 + i] = nullptr;
        }
    } else {
        for (size_t i = 0; i < tySize; i++) {
            _tmpSparseArrayValues[id - 1 + i] = nullptr;
        }
    }
}

} // END cg namespace
} // END CppAD namespace

#endif