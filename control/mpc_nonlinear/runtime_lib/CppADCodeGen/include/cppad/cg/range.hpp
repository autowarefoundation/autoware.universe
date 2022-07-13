#ifndef CPPAD_CG_RANGE_INCLUDED
#define CPPAD_CG_RANGE_INCLUDED
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
 * Combines ordered and non-overlapping regions of iteration indexes.
 * 
 * @param iterRegions the existing iteration regions
 * @param newIterRegions the iteration regions to be added
 */
inline void combineNonOverlapingIterationRanges(std::vector<size_t>& iterRegions,
                                                const std::vector<size_t>& newIterRegions) {
    if (iterRegions.empty()) {
        iterRegions = newIterRegions;
        return;
    } else if (newIterRegions.empty()) {
        return;
    }

    /**
     * regions are assumed to be ordered and non-overlapping
     */
    std::vector<size_t>::iterator itPos = iterRegions.begin();
    std::vector<size_t>::const_iterator itNew;
    for (itNew = newIterRegions.begin(); itNew != newIterRegions.end(); ++itNew) {
        size_t pos = *itNew;
        itPos = std::lower_bound(itPos, iterRegions.end(), pos);
        if (itPos == iterRegions.end()) {
            iterRegions.insert(iterRegions.end(), itNew, newIterRegions.end());
            break; // done
        } else if (*itPos == pos) {
            // same value -> must merge
            itPos = iterRegions.erase(itPos);
        } else {
            itPos = iterRegions.insert(itPos, pos);
        }
    }
    CPPADCG_ASSERT_UNKNOWN(iterRegions.size() % 2 == 0);
}

/**
 * Combines ordered regions of iteration indexes.
 * 
 * @param iterRegions the existing iteration regions
 * @param newIterRegions the iteration regions to be added
 */
inline void combineOverlapingIterationRanges(std::vector<size_t>& iterRegions,
                                             const std::vector<size_t>& newIterRegions) {
    if (iterRegions.empty()) {
        iterRegions = newIterRegions;
        return;
    } else if (newIterRegions.empty()) {
        return;
    }

    std::vector<std::pair<size_t, size_t> > sorted;
    for (size_t i = 0; i < iterRegions.size(); i += 2) {
        sorted.insert(sorted.end(), std::make_pair(iterRegions[i], iterRegions[i + 1]));
    }

    std::vector<std::pair<size_t, size_t> >::iterator begin = sorted.begin();
    for (size_t i = 0; i < newIterRegions.size(); i += 2) {
        std::pair<size_t, size_t> p(newIterRegions[i], newIterRegions[i + 1]);
        begin = std::lower_bound(begin, sorted.end(), p);
        begin = sorted.insert(begin, p);
        begin++;
    }

    std::vector<std::pair<size_t, size_t> > result;
    result.reserve(sorted.size());
    result.push_back(sorted[0]);
    for (size_t i = 1; i < sorted.size(); i++) {
        const std::pair<size_t, size_t>& curr = sorted[i]; // interval to be added
        std::pair<size_t, size_t>& top = result.back();

        if (top.second == (std::numeric_limits<size_t>::max)()) // avoid overflow
            break; // done, nothing can be added

        // if current interval is not overlapping with stack top,
        // push it to the stack
        if (top.second + 1 < curr.first) {
            result.push_back(curr);
        } else if (top.second < curr.second) {
            // Otherwise update the ending time of top if ending of current
            // interval is more
            top.second = curr.second;
        }
    }

    iterRegions.resize(result.size() * 2);
    for (size_t i = 0; i < result.size(); i++) {
        iterRegions[2 * i] = result[i].first;
        iterRegions[2 * i + 1] = result[i].second;
    }
}

/**
 * Determines the iteration regions not present in the provided iteration 
 * regions.
 * 
 * @param iterRegions a vector filled with ordered and non-overlapping 
 *                    iteration regions each defined with pairs of integers 
 *                    (start1, end1, start2, end2, ...)
 * @return the iterations not present in iterRegions
 */
inline std::vector<size_t> invertIterationRanges(const std::vector<size_t>& iterRegions) {
    std::vector<size_t> inverted;
    if (iterRegions.empty()) {
        inverted.resize(2);
        inverted[1] = (std::numeric_limits<size_t>::max)();
        return inverted;
    }

    CPPADCG_ASSERT_UNKNOWN(iterRegions.size() % 2 == 0);
    inverted.reserve(iterRegions.size() + 4);

    if (iterRegions[0] != 0) {
        inverted.push_back(0);
        inverted.push_back(iterRegions[0] - 1);
    }

    for (size_t i = 2; i < iterRegions.size(); i += 2) {
        CPPADCG_ASSERT_UNKNOWN(iterRegions[i - 1] < iterRegions[i]);
        inverted.push_back(iterRegions[i - 1] + 1);
        inverted.push_back(iterRegions[i] - 1);
    }

    if (iterRegions.back() != (std::numeric_limits<size_t>::max)()) {
        inverted.push_back(iterRegions.back() + 1);
        inverted.push_back((std::numeric_limits<size_t>::max)());
    }

    return inverted;
}

/**
 * Determines the iteration regions that an 'if' branch can be called for.
 * 
 * @param bScope a node that marks the beginning of the if branch. It must 
 *               be of type CGStartIfOp, CGElseIfOp or CGElseOp.
 * @param iterationIndexOp the iteration index node used in the condition(s)
 * @return the iteration regions
 */
template<class Base>
inline std::vector<size_t> ifBranchIterationRanges(const OperationNode<Base>* bScope,
                                                   IndexOperationNode<Base>*& iterationIndexOp) {
    CGOpCode bOp = bScope->getOperationType();

    if (bOp == CGOpCode::StartIf || bOp == CGOpCode::ElseIf) {
        OperationNode<Base>* cond = bScope->getArguments()[bOp == CGOpCode::StartIf ? 0 : 1].getOperation();
        CPPADCG_ASSERT_UNKNOWN(cond->getOperationType() == CGOpCode::IndexCondExpr);
        CPPADCG_ASSERT_UNKNOWN(cond->getArguments().size() == 1);
        CPPADCG_ASSERT_UNKNOWN(cond->getArguments()[0].getOperation() != nullptr);
        CPPADCG_ASSERT_UNKNOWN(cond->getArguments()[0].getOperation()->getOperationType() == CGOpCode::Index);
        iterationIndexOp = static_cast<IndexOperationNode<Base>*> (cond->getArguments()[0].getOperation());
        return cond->getInfo();

    } else {
        // else
        CPPADCG_ASSERT_UNKNOWN(bOp == CGOpCode::Else);

        std::vector<size_t> nonIterationRegions;
        OperationNode<Base>* ifBranch = bScope->getArguments()[0].getOperation();
        do {
            CGOpCode bbOp = ifBranch->getOperationType();
            OperationNode<Base>* cond = ifBranch->getArguments()[bbOp == CGOpCode::StartIf ? 0 : 1].getOperation();
            CPPADCG_ASSERT_UNKNOWN(cond->getOperationType() == CGOpCode::IndexCondExpr);
            CPPADCG_ASSERT_UNKNOWN(cond->getArguments().size() == 1);
            CPPADCG_ASSERT_UNKNOWN(cond->getArguments()[0].getOperation() != nullptr);
            CPPADCG_ASSERT_UNKNOWN(cond->getArguments()[0].getOperation()->getOperationType() == CGOpCode::Index);
            IndexOperationNode<Base>* indexOp = static_cast<IndexOperationNode<Base>*> (cond->getArguments()[0].getOperation());
            CPPADCG_ASSERT_UNKNOWN(iterationIndexOp == nullptr || iterationIndexOp == indexOp);
            iterationIndexOp = indexOp;

            combineOverlapingIterationRanges(nonIterationRegions, cond->getInfo());

            ifBranch = ifBranch->getArguments()[0].getOperation();
        } while (ifBranch->getOperationType() == CGOpCode::ElseIf);

        CPPADCG_ASSERT_UNKNOWN(iterationIndexOp != nullptr);

        // invert
        return invertIterationRanges(nonIterationRegions);
    }

}

} // END cg namespace
} // END CppAD namespace

#endif