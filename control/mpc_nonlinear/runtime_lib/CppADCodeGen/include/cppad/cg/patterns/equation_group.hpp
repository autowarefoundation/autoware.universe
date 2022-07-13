#ifndef CPPAD_CG_EQUATION_GROUP_INCLUDED
#define CPPAD_CG_EQUATION_GROUP_INCLUDED
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
 * A groups of equations which share common temporary variables
 */
template<class Base>
class EquationGroup {
public:
    /**
     * The equations in this group
     */
    std::set<EquationPattern<Base>*> equations;
    /**
     * Which dependents (equation indexes) must be evaluated at the same 
     * time due to shared indexed temporary variables
     */
    std::vector<std::set<size_t> > linkedDependents;
    /**
     * Which equation pattern should be evaluated at the same 
     * time due to shared non-indexed temporary variables
     */
    std::vector<std::set<EquationPattern<Base>*> > linkedEquationsByNonIndexedRel;
    std::set<EquationPattern<Base>*> linkedEquationsByNonIndexed; // only equation which do not have any shared indexed variables
    /**
     * The evaluated dependents in each loop iteration
     * ([iteration 1]{dep1, dep3, ...}; [iteration 2]{dep5, dep6, ...}; ...)
     */
    std::vector<std::set<size_t> > iterationDependents;
    /**
     * Reference iteration where all equations are present
     */
    size_t refIteration;
public:

    inline void findReferenceIteration() {
        CPPADCG_ASSERT_UNKNOWN(!iterationDependents.empty());

        for (size_t it = 0; it < iterationDependents.size(); it++) {
            if (iterationDependents[it].size() == equations.size()) {
                refIteration = it;
                return;
            }
        }

        CPPADCG_ASSERT_UNKNOWN(false);
    }

    inline long findIndexedLinkedDependent(size_t dep) const {
        size_t size = linkedDependents.size();
        for (size_t pos = 0; pos < size; pos++) {
            const std::set<size_t>& sameIndex = linkedDependents[pos];
            if (sameIndex.find(dep) != sameIndex.end()) {
                return pos;
            }
        }

        return -1;
    }

    inline size_t getLinkedEquationsByNonIndexedCount() const {
        return linkedEquationsByNonIndexed.size();
    }

    inline size_t findNonIndexedLinkedRel(EquationPattern<Base>* eq) const {
        size_t size = linkedEquationsByNonIndexedRel.size();
        for (size_t pos = 0; pos < size; pos++) {
            if (linkedEquationsByNonIndexedRel[pos].find(eq) != linkedEquationsByNonIndexedRel[pos].end()) {
                return pos;
            }
        }

        return -1;
    }

    inline void addLinkedEquationsByNonIndexed(EquationPattern<Base>* eq1,
                                               EquationPattern<Base>* eq2) {
        linkedEquationsByNonIndexed.insert(eq1);
        linkedEquationsByNonIndexed.insert(eq2);

        size_t size = linkedEquationsByNonIndexedRel.size();
        size_t pos1 = size;
        size_t pos2 = size;
        for (size_t pos = 0; pos < size; pos++) {
            if (linkedEquationsByNonIndexedRel[pos].find(eq1) != linkedEquationsByNonIndexedRel[pos].end()) {
                pos1 = pos;
                break;
            }
        }
        for (size_t pos = 0; pos < size; pos++) {
            if (linkedEquationsByNonIndexedRel[pos].find(eq2) != linkedEquationsByNonIndexedRel[pos].end()) {
                pos2 = pos;
                break;
            }
        }

        if (pos1 == pos2) {
            if (pos1 == size) {
                linkedEquationsByNonIndexedRel.resize(size + 1);
                linkedEquationsByNonIndexedRel[size].insert(eq1);
                linkedEquationsByNonIndexedRel[size].insert(eq2);
            }
        } else if (pos1 < size) {
            if (pos2 < size) {
                // must merge
                linkedEquationsByNonIndexedRel[pos1].insert(linkedEquationsByNonIndexedRel[pos2].begin(), linkedEquationsByNonIndexedRel[pos2].end());
                linkedEquationsByNonIndexedRel.erase(linkedEquationsByNonIndexedRel.begin() + pos2);
            } else {
                linkedEquationsByNonIndexedRel[pos1].insert(eq2);
            }

        } else {
            CPPADCG_ASSERT_UNKNOWN(pos2 < size);
            linkedEquationsByNonIndexedRel[pos2].insert(eq1);
        }
    }
};

} // END cg namespace
} // END CppAD namespace

#endif