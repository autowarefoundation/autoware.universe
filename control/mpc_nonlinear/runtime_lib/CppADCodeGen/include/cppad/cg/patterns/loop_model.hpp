#ifndef CPPAD_CG_LOOP_MODEL_INCLUDED
#define CPPAD_CG_LOOP_MODEL_INCLUDED
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
 * A model representing a loop body for source code generation
 *
 * @author Joao Leal
 */
template <class Base>
class LoopModel {
public:
    using CGB = CppAD::cg::CG<Base>;
    using Arg = Argument<Base>;
    using pairss = std::pair<size_t, size_t>;
public:
    static const std::string ITERATION_INDEX_NAME;
private:
    static const std::set<size_t> EMPTYSET;
private:
    const size_t loopId_;
    /**
     * The tape for a single loop iteration
     */
    ADFun<CGB>* const fun_;
    /**
     * Whether or not it calls atomic functions
     */
    const bool containsAtoms_;
    /**
     * Number of loop iterations
     */
    const size_t iterationCount_;
    /**
     * Loop tape dependent variable count (number of equation patterns)
     */
    const size_t m_;
    /**
     * The dependent variables ([tape equation][iteration])
     */
    std::vector<std::vector<LoopPosition> > dependentIndexes_;
    /**
     * The indexed independent variables ([tape variable][iteration])
     */
    std::vector<std::vector<LoopPosition> > indexedIndepIndexes_;
    /**
     * The non-indexed independent variables ([tape variable])
     */
    std::vector<LoopPosition> nonIndexedIndepIndexes_;
    /**
     * The independent variables related with temporary variables of the
     * original model.
     */
    std::vector<LoopPosition> temporaryIndependents_;
    /**
     * Maps the original dependent variable indexes to their positions in
     * the loop
     */
    std::map<size_t, LoopIndexedPosition> depOrigIndexes_;
    /**
     *
     */
    std::vector<IterEquationGroup<Base> > equationGroups_;
    std::vector<std::set<const IterEquationGroup<Base>*> > iteration2eqGroups_;
    /**
     * iteration -> original indep index -> tape indexes
     */
    std::vector<std::map<size_t, std::set<size_t> > > iteration2orig2indexedIndepIndexes_;
    /**
     * Maps the original variable indexes to non-indexed variables
     */
    std::map<size_t, LoopPosition*> orig2nonIndexedIndepIndexes_;
    /**
     * Maps the temporary variable indexes to the tape indexes
     */
    std::map<size_t, LoopPosition*> orig2tempIndepIndexes_;
    /**
     * index patterns for the indexed independent variables
     */
    std::vector<IndexPattern*> indepIndexPatterns_;
    /**
     * index pattern for the dependent variables
     */
    std::vector<IndexPattern*> depIndexPatterns_;
    /**
     * Jacobian sparsity pattern of the tape
     */
    std::vector<std::set<size_t> > jacTapeSparsity_;
    bool jacSparsity_;
    /**
     * Hessian sparsity pattern of the tape
     */
    std::vector<std::set<size_t> > hessTapeSparsity_;
    bool hessSparsity_;
public:

    /**
     * Creates a new atomic function that is responsible for defining the
     * dependencies to calls of a user atomic function.
     *
     * @param fun The tape for a single loop iteration (loop model)
     * @param containsAtoms Whether or not fun calls atomic functions
     * @param iterationCount Number of loop iterations
     * @param dependentOrigIndexes
     * @param indexedIndepOrigIndexes
     * @param nonIndexedIndepOrigIndexes
     * @param temporaryIndependents
     */
    LoopModel(ADFun<CGB>* fun,
              bool containsAtoms,
              size_t iterationCount,
              const std::vector<std::vector<size_t> >& dependentOrigIndexes,
              const std::vector<std::vector<size_t> >& indexedIndepOrigIndexes,
              const std::vector<size_t>& nonIndexedIndepOrigIndexes,
              const std::vector<size_t>& temporaryIndependents) :
        loopId_(createNewLoopId()),
        fun_(fun),
        containsAtoms_(containsAtoms),
        iterationCount_(iterationCount),
        m_(dependentOrigIndexes.size()),
        dependentIndexes_(m_, std::vector<LoopPosition>(iterationCount)),
        indexedIndepIndexes_(indexedIndepOrigIndexes.size(), std::vector<LoopPosition>(iterationCount)),
        nonIndexedIndepIndexes_(nonIndexedIndepOrigIndexes.size()),
        temporaryIndependents_(temporaryIndependents.size()),
        iteration2orig2indexedIndepIndexes_(iterationCount),
        jacSparsity_(false),
        hessSparsity_(false) {
        CPPADCG_ASSERT_KNOWN(fun != nullptr, "fun cannot be null");

        /**
         * dependents
         */
        for (size_t i = 0; i < m_; i++) {
            for (size_t it = 0; it < iterationCount_; it++) {
                size_t orig = dependentOrigIndexes[i][it];
                dependentIndexes_[i][it] = LoopPosition(i, orig);
                if (orig != (std::numeric_limits<size_t>::max)()) // some equations are not present in all iterations
                    depOrigIndexes_[orig] = LoopIndexedPosition(dependentIndexes_[i][it].tape,
                                                                dependentIndexes_[i][it].original,
                                                                it);
            }
        }

        /**
         * Must determine the equations which are present at the same iterations
         * (some equations may not be present at some iterations)
         */
        std::map<std::set<size_t>, std::set<size_t>, SetComparator<size_t> > iterations2equations;
        size_t lm = dependentIndexes_.size();

        for (size_t i = 0; i < lm; i++) {
            std::set<size_t> iterations;
            for (size_t it = 0; it < iterationCount_; it++) {
                if (dependentIndexes_[i][it].original != (std::numeric_limits<size_t>::max)()) {
                    iterations.insert(it);
                }
            }
            iterations2equations[iterations].insert(i);
        }

        equationGroups_.resize(iterations2equations.size());
        iteration2eqGroups_.resize(iterationCount_);

        std::map<std::set<size_t>, std::set<size_t>, SetComparator<size_t> >::const_iterator itEqeIt;
        size_t g = 0;
        for (itEqeIt = iterations2equations.begin(); itEqeIt != iterations2equations.end(); ++itEqeIt, g++) {
            const std::set<size_t>& iterations = itEqeIt->first;

            IterEquationGroup<Base>& group = equationGroups_[g];
            group.index = g;
            group.tapeI = itEqeIt->second;
            group.iterations = iterations;
            group.model = this;

            // map iterations to the equation groups
            for (size_t itIt : iterations) {
                iteration2eqGroups_[itIt].insert(&group);
            }
        }

        /**
         * independents
         */
        size_t nIndexed = indexedIndepOrigIndexes.size();

        // indexed
        for (size_t it = 0; it < iterationCount_; it++) {
            for (size_t j = 0; j < nIndexed; j++) {
                size_t orig = indexedIndepOrigIndexes[j][it];
                indexedIndepIndexes_[j][it] = LoopPosition(j, orig);
                if (orig != (std::numeric_limits<size_t>::max)()) //some variables are not present in all iterations
                    iteration2orig2indexedIndepIndexes_[it][orig].insert(j);
            }
        }

        // non-indexed
        size_t nNonIndexed = nonIndexedIndepOrigIndexes.size();
        for (size_t j = 0; j < nNonIndexed; j++) {
            size_t orig = nonIndexedIndepOrigIndexes[j];
            nonIndexedIndepIndexes_[j] = LoopPosition(nIndexed + j, orig);
            orig2nonIndexedIndepIndexes_[orig] = &nonIndexedIndepIndexes_[j];
        }

        // temporary
        for (size_t j = 0; j < temporaryIndependents.size(); j++) {
            size_t k = temporaryIndependents[j];
            temporaryIndependents_[j] = LoopPosition(nIndexed + nNonIndexed + j, k);
            orig2tempIndepIndexes_[k] = &temporaryIndependents_[j];
        }
    }

    LoopModel(const LoopModel<Base>&) = delete;
    LoopModel& operator=(const LoopModel<Base>&) = delete;

    /**
     * Provides a unique identifier for this loop.
     *
     * @return a unique identifier ID
     */
    inline size_t getLoopId() const {
        return loopId_;
    }

    /**
     * Whether or not the tape for the loop calls atomic functions.
     *
     * @return Whether or not it calls atomic functions.
     */
    inline bool isContainsAtomics() const {
        return containsAtoms_;
    }

    /**
     * Provides the number of iterations in the loop
     *
     * @return the number of iterations in the loop
     */
    inline const size_t getIterationCount() const {
        return iterationCount_;
    }

    /**
     * Provides the tape that represents the loop model
     *
     * @return the tape of the loop model
     */
    inline ADFun<CGB>& getTape() const {
        return *fun_;
    }

    /**
     * Provides the number of dependent variables in the loop tape/model
     * (number of equation patterns).
     *
     * @return the number of dependents in the loop model
     *         (number of equation patterns)
     */
    inline size_t getTapeDependentCount() const {
        return m_;
    }

    /**
     * Provides the number of independent variables in the loop tape/model
     * (number of indexed + non-indexed + temporary variables).
     *
     * @return the number of independents in the loop model
     */
    inline size_t getTapeIndependentCount() const {
        return fun_->Domain();
    }

    /**
     * Provides the dependent variables indexes ([tape equation][iteration])
     */
    inline const std::vector<std::vector<LoopPosition> >& getDependentIndexes() const {
        return dependentIndexes_;
    }

    /**
     * Provides groups of equations present at the same iterations
     */
    inline const std::vector<IterEquationGroup<Base> >& getEquationsGroups() const {
        return equationGroups_;
    }

    inline const std::vector<std::set<const IterEquationGroup<Base>*> >& getIterationEquationsGroup() const {
        return iteration2eqGroups_;
    }

    /**
     * Provides the indexed independent variables ([tape variable][iteration])
     */
    inline const std::vector<std::vector<LoopPosition> >& getIndexedIndepIndexes() const {
        return indexedIndepIndexes_;
    }

    /**
     * Provides the non-indexed independent variables ([tape variable])
     */
    inline const std::vector<LoopPosition>& getNonIndexedIndepIndexes() const {
        return nonIndexedIndepIndexes_;
    }

    /**
     * Provides the independent variables related with temporary variables of the
     * original model.
     */
    inline const std::vector<LoopPosition>& getTemporaryIndependents() const {
        return temporaryIndependents_;
    }

    /**
     * Provides the locations where a dependent variable is used
     *
     * @param origI the dependent variable index in the original model
     * @return the locations where a dependent variable is used
     */
    inline const LoopIndexedPosition& getTapeDependentIndex(size_t origI) const {
        return depOrigIndexes_.at(origI);
    }

    inline const std::map<size_t, LoopIndexedPosition>& getOriginalDependentIndexes() const {
        return depOrigIndexes_;
    }

    /**
     * Maps the original variable indexes to non-indexed variables
     */
    inline const LoopPosition* getNonIndexedIndepIndexes(size_t origJ) const {
        std::map<size_t, LoopPosition*>::const_iterator it = orig2nonIndexedIndepIndexes_.find(origJ);
        if (it != orig2nonIndexedIndepIndexes_.end()) {
            return it->second;
        } else {
            return nullptr;
        }
    }

    /**
     * Maps the temporary variable indexes to temporary variables
     */
    inline const LoopPosition* getTempIndepIndexes(size_t k) const {
        std::map<size_t, LoopPosition*>::const_iterator it = orig2tempIndepIndexes_.find(k);
        if (it != orig2tempIndepIndexes_.end()) {
            return it->second;
        } else {
            return nullptr;
        }
    }

    /**
     * Finds the local tape variable indexes which use a given model
     * variable at a given iteration
     *
     * @param origJ the index of the variable in the original model
     * @param iteration the iteration
     * @return the indexes of tape variables where the variable is used
     */
    inline const std::set<size_t>& getIndexedTapeIndexes(size_t iteration, size_t origJ) const {
        CPPADCG_ASSERT_UNKNOWN(iteration < iteration2orig2indexedIndepIndexes_.size());

        const std::map<size_t, std::set<size_t> >& itOrigs = iteration2orig2indexedIndepIndexes_[iteration];
        std::map<size_t, std::set<size_t> >::const_iterator it = itOrigs.find(origJ);
        if (it != itOrigs.end()) {
            return it->second;
        } else {
            return EMPTYSET;
        }
    }

    /**
     * Finds the local tape variable indexes which use a given model variable
     *
     * @param origJ the index of the variable in the original model
     * @return all the indexed tape variables for each iteration where the
     *         variable is used
     */
    inline std::map<size_t, std::set<size_t> > getIndexedTapeIndexes(size_t origJ) const {
        std::map<size_t, std::set<size_t> > iter2TapeJs;

        for (size_t iter = 0; iter < iterationCount_; iter++) {
            const std::map<size_t, std::set<size_t> >& itOrigs = iteration2orig2indexedIndepIndexes_[iter];
            std::map<size_t, std::set<size_t> >::const_iterator it = itOrigs.find(origJ);
            if (it != itOrigs.end()) {
                iter2TapeJs[iter] = it->second;
            }
        }

        return iter2TapeJs;
    }

    inline void detectIndexPatterns() {
        if (indepIndexPatterns_.size() > 0)
            return; // already done

        indepIndexPatterns_.resize(indexedIndepIndexes_.size());
        for (size_t j = 0; j < indepIndexPatterns_.size(); j++) {
            std::map<size_t, size_t> indexes;
            for (size_t it = 0; it < iterationCount_; it++) {
                size_t orig = indexedIndepIndexes_[j][it].original;
                if (orig != (std::numeric_limits<size_t>::max)()) // some variables are not present in all iteration
                    indexes[it] = orig;
            }
            indepIndexPatterns_[j] = IndexPattern::detect(indexes);
        }

        depIndexPatterns_.resize(dependentIndexes_.size());
        for (size_t j = 0; j < depIndexPatterns_.size(); j++) {
            std::map<size_t, size_t> indexes;
            for (size_t it = 0; it < iterationCount_; it++) {
                size_t e = dependentIndexes_[j][it].original;
                if (e != (std::numeric_limits<size_t>::max)()) // some equations are not present in all iteration
                    indexes[it] = e;
            }

            depIndexPatterns_[j] = IndexPattern::detect(indexes);
        }
    }

    inline const std::vector<IndexPattern*>& getDependentIndexPatterns() const {
        return depIndexPatterns_;
    }

    inline const std::vector<IndexPattern*>& getIndependentIndexPatterns() const {
        return indepIndexPatterns_;
    }

    inline bool isTemporary(size_t tapeJ) const {
        size_t nIndexed = indexedIndepIndexes_.size();
        size_t nNonIndexed = nonIndexedIndepIndexes_.size();

        return nIndexed + nNonIndexed <= tapeJ;
    }

    inline bool isIndexedIndependent(size_t tapeJ) const {
        return tapeJ < indexedIndepIndexes_.size();
    }

    inline void evalJacobianSparsity() {
        if (!jacSparsity_) {
            jacTapeSparsity_ = jacobianSparsitySet<std::vector<std::set<size_t> >, CGB>(*fun_);
            jacSparsity_ = true;
        }
    }

    inline const std::vector<std::set<size_t> >& getJacobianSparsity() const {
        return jacTapeSparsity_;
    }

    inline void evalHessianSparsity() {
        if (!hessSparsity_) {
            size_t n = fun_->Domain();
            hessTapeSparsity_.resize(n);

            for (size_t g = 0; g < equationGroups_.size(); g++) {
                equationGroups_[g].evalHessianSparsity();
                const std::vector<std::set<size_t> >& ghess = equationGroups_[g].getHessianSparsity();
                for (size_t j = 0; j < n; j++) {
                    hessTapeSparsity_[j].insert(ghess[j].begin(), ghess[j].end());
                }
            }

            hessSparsity_ = true;
        }
    }

    inline const std::vector<std::set<size_t> >& getHessianSparsity() const {
        return hessTapeSparsity_;
    }

    virtual ~LoopModel() {
        delete fun_;
        for (size_t i = 0; i < indepIndexPatterns_.size(); i++) {
            delete indepIndexPatterns_[i];
        }
        for (size_t i = 0; i < depIndexPatterns_.size(); i++) {
            delete depIndexPatterns_[i];
        }
    }

    static inline void printOriginalVariableIndexes(std::ostringstream& ss,
                                                    const std::vector<LoopPosition>& indexes) {
        for (size_t iter = 0; iter < indexes.size(); iter++) {
            if (iter > 0) ss << ", ";
            ss << indexes[iter].original;
        }
    }

private:

    static size_t createNewLoopId() {
        CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;
        static size_t count = 0;
        count++;
        return count;
    }

};

template<class Base>
const std::string LoopModel<Base>::ITERATION_INDEX_NAME("j");

template<class Base>
const std::set<size_t> LoopModel<Base>::EMPTYSET;

} // END cg namespace
} // END CppAD namespace

#endif
