#ifndef CPPAD_CG_LANGUAGE_LATEX_ARRAYS_INCLUDED
#define CPPAD_CG_LANGUAGE_LATEX_ARRAYS_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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

template<class Base>
void LanguageLatex<Base>::printArrayCreationOp(OperationNode<Base>& array) {
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
                _code << _startAlgLine << _startEq
                        << auxArrayName_ << _assignStr << _nameGen->generateTemporaryArray(array, getVariableID(array))
                        << _endEq << _endAlgLine
                        << " % size: " << args.size() << _endline;
                firstElement = false;
            }

            // try to use a loop for element assignment
            size_t newI = printArrayCreationUsingLoop(startPos, array, i, _tmpArrayValues);

            // print elements not assign in a loop
            if (newI == i) {
                // individual element assignment
                _code << _startAlgLine << _startEq
                        << auxArrayName_ << "[" << i << "]" << _assignStr;
                print(args[i]);
                _code << _endEq << _endAlgLine << _endline;

                _tmpArrayValues[startPos + i] = &args[i];

            } else {
                i = newI - 1;
            }
        }
    }
}

template<class Base>
void LanguageLatex<Base>::printSparseArrayCreationOp(OperationNode<Base>& array) {
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
                _code << _startAlgLine << _startEq
                        << auxArrayName_ << _assignStr << _nameGen->generateTemporarySparseArray(array, getVariableID(array))
                        << _endEq << _endAlgLine
                        << " % nnz: " << args.size() << "  size:" << info[0] << _endline;
                firstElement = false;
            }

            // try to use a loop for element assignment
            size_t newI = printArrayCreationUsingLoop(startPos, array, i, _tmpSparseArrayValues);

            // print element values not assign in a loop
            if (newI == i) {
                // individual element assignment
                _code << _startAlgLine << _startEq
                        << auxArrayName_ << "[" << i << "]" << _assignStr;
                print(args[i]);
                _code << _endEq;
                _code << ", ";
                // print indexes (location of values)
                _code << _startEq
                        << _C_SPARSE_INDEX_ARRAY << "[";
                if (startPos != 0) _code << startPos << "+";
                _code << i << "]" << _assignStr << info[i + 1]
                        << _endEq << _endAlgLine << _endline;

                _tmpSparseArrayValues[startPos + i] = &args[i];

            } else {
                // print indexes (location of values)
                for (size_t j = i; j < newI; j++) {
                    _code << _startAlgLine << _startEq
                            << _C_SPARSE_INDEX_ARRAY << "[";
                    if (startPos != 0) _code << startPos << "+";
                    _code << j << "]" << _assignStr << info[j + 1]
                            << _endEq << _endAlgLine << _endline;
                }

                i = newI - 1;
            }


        } else {
            // print indexes (location of values)
            _code << _startAlgLine << _startEq
                    << _C_SPARSE_INDEX_ARRAY << "[";
            if (startPos != 0) _code << startPos << "+";
            _code << i << "]" << _assignStr << info[i + 1]
                    << _endEq << _endAlgLine << _endline;
        }

    }
}

template<class Base>
inline size_t LanguageLatex<Base>::printArrayCreationUsingLoop(size_t startPos,
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
                arrayAssign << indep << "[i]";
            else
                arrayAssign << indep << "[" << offset << " + i]";

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

            auto* lip2 = new LinearIndexPattern(*refLIp);
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
    _code << _forStart << "{$i \\in \\left[" <<  starti << ", " << (i - 1) << "\\right]$}" << _endline;
    _indentationLevel++;
    _code << _startAlgLine << _startEq
            << auxArrayName_ << "[i]" << _assignStr << arrayAssign.str()
            << _endEq << _endAlgLine << _endline;
    _indentationLevel--;
    _code << _forEnd << _endline;

    /**
     * update values in the global temporary array
     */
    for (size_t ii = starti; ii < i; ii++) {
        tmpArrayValues[startPos + ii] = &args[ii];
    }

    return i;
}

template<class Base>
inline std::string LanguageLatex<Base>::getTempArrayName(const OperationNode<Base>& op) {
    if (op.getOperationType() == CGOpCode::ArrayCreation)
        return _nameGen->generateTemporaryArray(op);
    else
        return _nameGen->generateTemporarySparseArray(op);
}

template<class Base>
void LanguageLatex<Base>::printArrayElementOp(OperationNode<Base>& op) {
    CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for array element operation")
    CPPADCG_ASSERT_KNOWN(op.getArguments()[0].getOperation() != nullptr, "Invalid argument for array element operation")
    CPPADCG_ASSERT_KNOWN(op.getInfo().size() == 1, "Invalid number of information indexes for array element operation")

    OperationNode<Base>& arrayOp = *op.getArguments()[0].getOperation();
    std::string arrayName;
    if (arrayOp.getOperationType() == CGOpCode::ArrayCreation)
        arrayName = _nameGen->generateTemporaryArray(arrayOp, getVariableID(arrayOp));
    else
        arrayName = _nameGen->generateTemporarySparseArray(arrayOp, getVariableID(arrayOp));

    _code << "(" << arrayName << ")[" << op.getInfo()[0] << "]";
}

template<class Base>
inline void LanguageLatex<Base>::printArrayStructInit(const std::string& dataArrayName,
                                                      size_t pos,
                                                      const std::vector<OperationNode<Base>*>& arrays,
                                                      size_t k) {
    _ss.str("");
    _ss << dataArrayName << "[" << pos << "]";
    printArrayStructInit(_ss.str(), *arrays[k]);
}

template<class Base>
inline void LanguageLatex<Base>::printArrayStructInit(const std::string& dataArrayName,
                                                      OperationNode<Base>& array) {
    /**
     * TODO: finish this
     */
    const std::string& aName = createVariableName(array);

    if (array.getOperationType() == CGOpCode::ArrayCreation) {
        // dense array
        size_t size = array.getArguments().size();
        if (size > 0)
            _code << dataArrayName << ".data = " << aName << "; ";
        else
            _code << dataArrayName << ".data = NULL; ";
        _code << dataArrayName << ".size = " << size << "; "
                << dataArrayName << ".sparse = " << false << ";";
    } else {
        // sparse array
        CPPADCG_ASSERT_KNOWN(array.getOperationType() == CGOpCode::SparseArrayCreation, "Invalid node type")
        size_t nnz = array.getArguments().size();
        if (nnz > 0)
            _code << dataArrayName << ".data = " << aName << "; ";
        else
            _code << dataArrayName << ".data = NULL; ";
        _code << dataArrayName << ".size = " << array.getInfo()[0] << "; "
                << dataArrayName << ".sparse = " << true << "; "
                << dataArrayName << ".nnz = " << nnz << "; ";
        if (nnz > 0) {
            size_t id = getVariableID(array);
            _code << dataArrayName << ".idx = &(" << _C_SPARSE_INDEX_ARRAY << "[" << (id - 1) << "]);";
        }
    }
    _code << _endline;
}

template<class Base>
inline void LanguageLatex<Base>::markArrayChanged(OperationNode<Base>& ty) {
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