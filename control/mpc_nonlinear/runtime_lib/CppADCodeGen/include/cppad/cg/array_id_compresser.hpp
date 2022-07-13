#ifndef CPPAD_CG_ARRAY_COMPRESSER_INCLUDED
#define CPPAD_CG_ARRAY_COMPRESSER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
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
 * Arrays in generated source can reuse space from a global array.
 * Positions for each array can be determined using this class.
 * Array locations are stored in node IDs as: id = location + 1.
 */
template<class Base>
class ArrayIdCompresser {
private:
    /**
     * [start] = end
     */
    std::map<size_t, size_t> _freeArrayStartSpace;
    /**
     * [end] = start
     */
    std::map<size_t, size_t> _freeArrayEndSpace;
    /**
     * values in temporary array
     */
    std::vector<const Argument<Base>*> _tmpArrayValues;
    /**
     * Variable IDs
     */
    CodeHandlerVector<Base, size_t>& _varId;
    /**
     * Maximum array id
     */
    size_t _idArrayCount;
public:

    /**
     * Creates an ArrayIdCompresser
     * @param maxArraySize The likely number of elements in the temporary array
     */
    inline ArrayIdCompresser(CodeHandlerVector<Base, size_t>& varId,
                             size_t maxArraySize) :
        _tmpArrayValues(maxArraySize, nullptr),
        _varId(varId),
        _idArrayCount(1) {
    }

    inline size_t getIdCount() const {
        return _idArrayCount;
    }

    inline void addFreeArraySpace(const OperationNode<Base>& released) {
        size_t arrayStart = _varId[released] - 1;
        const size_t arraySize = released.getArguments().size();
        if (arraySize == 0)
            return; // nothing to do (no free space)
        size_t arrayEnd = arrayStart + arraySize - 1;

        std::map<size_t, size_t>::iterator it;
        if (arrayStart > 0) {
            // try to merge with previous free space
            it = _freeArrayEndSpace.find(arrayStart - 1); // previous
            if (it != _freeArrayEndSpace.end()) {
                arrayStart = it->second; // merge space
                _freeArrayEndSpace.erase(it);
                _freeArrayStartSpace.erase(arrayStart);
            }
        }

        // try to merge with the next free space
        it = _freeArrayStartSpace.find(arrayEnd + 1); // next
        if (it != _freeArrayStartSpace.end()) {
            arrayEnd = it->second; // merge space 
            _freeArrayStartSpace.erase(it);
            _freeArrayEndSpace.erase(arrayEnd);
        }

        _freeArrayStartSpace[arrayStart] = arrayEnd;
        _freeArrayEndSpace[arrayEnd] = arrayStart;

        CPPADCG_ASSERT_UNKNOWN(_freeArrayStartSpace.size() == _freeArrayEndSpace.size());
    }

    inline size_t reserveArraySpace(const OperationNode<Base>& newArray) {
        size_t arraySize = newArray.getArguments().size();

        if (arraySize == 0)
            return 0; // nothing to do (no space required)

        std::set<size_t> blackList;
        const std::vector<Argument<Base> >& args = newArray.getArguments();
        for (size_t i = 0; i < args.size(); i++) {
            const OperationNode<Base>* argOp = args[i].getOperation();
            if (argOp != nullptr && argOp->getOperationType() == CGOpCode::ArrayElement) {
                const OperationNode<Base>& otherArray = *argOp->getArguments()[0].getOperation();
                CPPADCG_ASSERT_UNKNOWN(_varId[otherArray] > 0); // make sure it had already been assigned space
                size_t otherArrayStart = _varId[otherArray] - 1;
                size_t index = argOp->getInfo()[0];
                blackList.insert(otherArrayStart + index);
            }
        }

        /**
         * Find the best location for the new array
         */
        std::map<size_t, size_t>::reverse_iterator it;
        std::map<size_t, size_t>::reverse_iterator itBestFit = _freeArrayStartSpace.rend();
        size_t bestCommonValues = 0; // the number of values likely to be the same
        for (it = _freeArrayStartSpace.rbegin(); it != _freeArrayStartSpace.rend(); ++it) {
            size_t start = it->first;
            size_t end = it->second;
            size_t space = end - start + 1;
            if (space < arraySize) {
                continue;
            }

            std::set<size_t>::const_iterator itBlack = blackList.lower_bound(start);
            if (itBlack != blackList.end() && *itBlack <= end) {
                continue; // cannot use this space
            }

            //possible candidate
            if (itBestFit == _freeArrayStartSpace.rend()) {
                itBestFit = it;
            } else {
                size_t bestSpace = itBestFit->second - itBestFit->first + 1;

                size_t commonVals = 0;
                for (size_t i = 0; i < arraySize; i++) {
                    if (isSameArrayElement(_tmpArrayValues[start + i], args[i])) {
                        commonVals++;
                    }
                }

                if (space < bestSpace || commonVals > bestCommonValues) {
                    // better fit
                    itBestFit = it;
                    bestCommonValues = commonVals;
                    if (bestCommonValues == arraySize) {
                        break; // jackpot
                    }
                }
            }
        }

        size_t bestStart = (std::numeric_limits<size_t>::max)();
        if (itBestFit != _freeArrayStartSpace.rend()) {
            /**
             * Use available space
             */
            bestStart = itBestFit->first;
            size_t bestEnd = itBestFit->second;
            size_t bestSpace = bestEnd - bestStart + 1;
            _freeArrayStartSpace.erase(bestStart);
            if (bestSpace == arraySize) {
                // entire space 
                _freeArrayEndSpace.erase(bestEnd);
            } else {
                // some space left
                size_t newFreeStart = bestStart + arraySize;
                _freeArrayStartSpace[newFreeStart] = bestEnd;
                _freeArrayEndSpace.at(bestEnd) = newFreeStart;
            }

        } else {
            /**
             * no space available, need more
             */
            // check if there is some free space at the end
            std::map<size_t, size_t>::iterator itEnd;
            itEnd = _freeArrayEndSpace.find(_idArrayCount - 1 - 1); // IDcount - initialID - 1
            if (itEnd != _freeArrayEndSpace.end()) {
                // check if it can be used
                size_t lastSpotStart = itEnd->second;
                size_t lastSpotEnd = itEnd->first;
                size_t lastSpotSize = lastSpotEnd - lastSpotStart + 1;
                std::set<size_t>::const_iterator itBlack = blackList.lower_bound(lastSpotStart);
                if (itBlack == blackList.end()) {
                    // can use this space
                    _freeArrayEndSpace.erase(itEnd);
                    _freeArrayStartSpace.erase(lastSpotStart);

                    _idArrayCount += arraySize - lastSpotSize;
                    bestStart = lastSpotStart;
                }
            }

            if (bestStart == (std::numeric_limits<size_t>::max)()) {
                // brand new space
                size_t id = _idArrayCount;
                _idArrayCount += arraySize;
                bestStart = id - 1;
            }
        }

        for (size_t i = 0; i < arraySize; i++) {
            _tmpArrayValues[bestStart + i] = &args[i];
        }

        CPPADCG_ASSERT_UNKNOWN(_freeArrayStartSpace.size() == _freeArrayEndSpace.size());

        return bestStart;
    }

    inline static bool isSameArrayElement(const Argument<Base>* oldArg,
                                          const Argument<Base>& arg) {
        if (oldArg != nullptr) {
            if (oldArg->getParameter() != nullptr) {
                if (arg.getParameter() != nullptr) {
                    return (*arg.getParameter() == *oldArg->getParameter());
                }
            } else {
                return (arg.getOperation() == oldArg->getOperation());
            }
        }
        return false;
    }

    virtual ~ArrayIdCompresser() {
    }
};

} // END cg namespace
} // END CppAD namespace

#endif