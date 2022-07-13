#ifndef CPPAD_CG_LOOP_INCLUDED
#define CPPAD_CG_LOOP_INCLUDED
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

template<class Base>
class IndependentOrder {
public:
    // provides an independent variable for each loop iteration
    const std::vector<const OperationNode<Base>*> order;

    inline IndependentOrder(const std::vector<const OperationNode<Base>*>& myOrder) :
        order(myOrder) {
    }
};

template<class Base>
class OperationArgumentsIndepOrder {
public:
    std::map<size_t, IndependentOrder<Base>*> arg2Order;
};

/**
 *
 */
template<class Base>
class LoopCodeHandler : public CodeHandler<Base> {
public:
    using CodeHandler<Base>::manageOperationNode;

};

/**
 * A for loop.
 */
template<class Base>
class Loop {
private:
    using IndexValue = std::pair<size_t, CG<Base> >;
public:
    /**
     * The equations inside the loop
     */
    std::set<EquationPattern<Base>*> equations;
    /**
     * Which argument positions of operations (from the reference dependent)
     * use indexed independent variables
     * (operation -> argument index -> iteration -> independent)
     */
    IndexedIndependent<Base> indexedOpIndep;
private:
    /**
     * Code handler used to generate the original operation graph
     */
    CodeHandler<Base>* handlerOrig_;
    /**
     *
     */
    std::unique_ptr<CodeHandlerVector<Base, size_t>> varId_;
    /**
     * Which variables depend on indexed independents
     */
    std::unique_ptr<CodeHandlerVector<Base, bool>> varIndexed_;
    /**
     * The number of iterations this loop will have
     */
    size_t iterationCount_;
    /**
     * Code handler for the operations in the loop
     */
    LoopCodeHandler<Base> handler_;
    /**
     * Groups of equations which share common temporary variables
     */
    std::vector<EquationGroup<Base> > eqGroups_;
    /**
     * The order of equation patterns in the loop's tape
     */
    std::map<EquationPattern<Base>*, size_t> equationOrder_;
    /**
     * The evaluated dependents in each loop iteration
     * ([iteration 1]{dep1, dep3, ...}; [iteration 2]{dep5, dep6, ...}; ...)
     */
    std::vector<std::set<size_t> > iterationDependents_;
    /**
     * Map each dependent to its corresponding iteration
     */
    std::map<size_t, size_t> dep2Iteration_;
    /**
     * total number of independent variables in the loop's tape
     */
    size_t nIndependents_;
    /**
     *
     */
    LoopModel<Base>* loopModel_;
    /**
     * indexed independent variables for the loop (clones)
     */
    std::map<const OperationNode<Base>*, IndexValue> independentsIndexed_;
    /**
     * non-indexed independent variables for the loop (clones)
     */
    std::map<const OperationNode<Base>*, IndexValue> independentsNonIndexed_;
    /**
     * independent variables for the loop created from temporary variables (clones)
     */
    std::map<const OperationNode<Base>*, IndexValue> independentsTemp_;
    /**
     * id -> clone
     */
    std::map<size_t, OperationNode<Base>*> clonesTemporary_;
    /**
     * orig -> clone
     */
    std::map<const OperationNode<Base>*, OperationNode<Base>*> temporaryClone2Orig_;
    /**
     * orig -> clone
     */
    std::map<const OperationNode<Base>*, OperationNode<Base>*> orig2ConstIndepClone_;
    /**
     * independent order -> clone
     */
    std::map<const IndependentOrder<Base>*, OperationNode<Base>*> indexedIndep2clone_;
    /**
     * Maps the independent from the first loop iteration to its independent
     * variable order
     * (first independent in the order -> all possible orders)
     */
    std::map<const OperationNode<Base>*, std::vector<IndependentOrder<Base>*> > firstIndep2orders_;
    /**
     * Maps an operations to their arguments and the corresponding
     * independent variable order
     */
    std::map<const OperationNode<Base>*, OperationArgumentsIndepOrder<Base>* > op2Arg2IndepOrder_;
    /**
     * used just to keep pointers so that the objects can be deleted later
     */
    std::forward_list<OperationArgumentsIndepOrder<Base>*> arg2IndepOrder_;
    /**
     * Variable id counter
     */
    size_t idCounter_;
public:

    inline Loop(EquationPattern<Base>& eq) :
        handlerOrig_(eq.depRef.getCodeHandler()),
        varId_(handlerOrig_ != nullptr ? new CodeHandlerVector<Base, size_t>(*handlerOrig_): nullptr),
        varIndexed_(handlerOrig_ != nullptr ? new CodeHandlerVector<Base, bool>(*handlerOrig_): nullptr),
        iterationCount_(0), // not known yet (only after all equations have been added)
        eqGroups_(1),
        nIndependents_(0),
        loopModel_(nullptr),
        idCounter_(0) { // not really required (but it avoids warnings)
        //indexedOpIndep(eq.indexedOpIndep), // must be determined later for a different reference
        //constOperationIndependents(eq.constOperationIndependents) {
        equations.insert(&eq);
        eqGroups_[0].equations.insert(&eq);

        if(varId_ != nullptr)
            varId_->adjustSize();
    }

    Loop(const Loop<Base>& other) = delete;
    Loop& operator=(const Loop<Base>& rhs) = delete;

    inline void addEquation(EquationPattern<Base>& eq) {
        equations.insert(&eq);
        eqGroups_[0].equations.insert(&eq);
    }

    inline void setLinkedDependents(const std::set<std::set<size_t>*>& newLoopRelations) {
        CPPADCG_ASSERT_UNKNOWN(eqGroups_.size() == 1);

        eqGroups_[0].linkedDependents.clear();
        eqGroups_[0].linkedDependents.reserve(newLoopRelations.size());

        for (std::set<size_t>* it : newLoopRelations)
            eqGroups_[0].linkedDependents.push_back(*it);
    }

    inline void addLinkedEquationsByNonIndexed(EquationPattern<Base>* eq1,
                                               EquationPattern<Base>* eq2) {
        eqGroups_[0].addLinkedEquationsByNonIndexed(eq1, eq2);
    }

    inline size_t getLinkedEquationsByNonIndexedCount() const {
        return eqGroups_[0].getLinkedEquationsByNonIndexedCount();
    }

    /**
     * The number of iterations this loop will have
     */
    inline size_t getIterationCount() const {
        return iterationCount_;
    }

    const std::vector<std::set<size_t> >& getIterationDependents() const {
        return iterationDependents_;
    }

    inline LoopModel<Base>* getModel() const {
        return loopModel_;
    }

    inline LoopModel<Base>* releaseLoopModel() {
        LoopModel<Base>* loopAtomic = loopModel_;
        loopModel_ = nullptr;
        return loopAtomic;
    }

    /**
     * Combines the provided loop with the current one
     *
     * @param other The other loop
     */
    void merge(Loop<Base>& other,
               const std::set<EquationPattern<Base>*>& indexedLoopRelations,
               const std::vector<std::pair<EquationPattern<Base>*, EquationPattern<Base>*> >& nonIndexedLoopRelations) {
        CPPADCG_ASSERT_UNKNOWN(iterationCount_ == other.iterationCount_)


        equations.insert(other.equations.begin(), other.equations.end());
        other.equations.clear(); // so that it does not delete the equations

        CPPADCG_ASSERT_UNKNOWN(eqGroups_.size() == 1)
        CPPADCG_ASSERT_UNKNOWN(other.eqGroups_.size() == 1)

        EquationGroup<Base>& g = eqGroups_[0];
        EquationGroup<Base>& og = other.eqGroups_[0];
        g.equations.insert(og.equations.begin(), og.equations.end());

        CPPADCG_ASSERT_UNKNOWN(equationOrder_.empty())
        CPPADCG_ASSERT_UNKNOWN(iterationDependents_.empty())

        g.linkedEquationsByNonIndexed.insert(og.linkedEquationsByNonIndexed.begin(), og.linkedEquationsByNonIndexed.end());
        for (EquationPattern<Base>* itNIndexed : indexedLoopRelations) {
            g.linkedEquationsByNonIndexed.erase(itNIndexed);
        }

        for (size_t e = 0; e < nonIndexedLoopRelations.size(); e++) {
            g.addLinkedEquationsByNonIndexed(nonIndexedLoopRelations[e].first, nonIndexedLoopRelations[e].second);
        }

    }

    void mergeEqGroups(Loop<Base>& other) {
        eqGroups_.insert(eqGroups_.end(), other.eqGroups_.begin(), other.eqGroups_.end());

        size_t nEq = equations.size();
        equations.insert(other.equations.begin(), other.equations.end());
        other.equations.clear(); // so that it does not delete the equations

        /**
         * Update equation index
         */
        for (const auto& it : other.equationOrder_) {
            equationOrder_[it.first] = it.second + nEq;
        }

        CPPADCG_ASSERT_UNKNOWN(iterationDependents_.size() == other.iterationDependents_.size())
        for (size_t iter = 0; iter < iterationDependents_.size(); iter++) {
            iterationDependents_[iter].insert(other.iterationDependents_[iter].begin(), other.iterationDependents_[iter].end());
        }

    }

    void createLoopModel(const std::vector<CG<Base> >& dependents,
                         const std::vector<CG<Base> >& independents,
                         const std::map<size_t, EquationPattern<Base>*>& dep2Equation,
                         std::map<OperationNode<Base>*, size_t>& origTemp2Index) {

        CPPADCG_ASSERT_UNKNOWN(dep2Iteration_.empty())
        for (size_t iter = 0; iter < iterationCount_; iter++) {
            const std::set<size_t>& deps = iterationDependents_[iter];

            for (size_t d : deps) {
                dep2Iteration_[d] = iter;
            }
        }

        if(varIndexed_ != nullptr) {
            varIndexed_->adjustSize();
            varIndexed_->fill(false);
        }

        /**
         * Determine the reference iteration for each
         */
        for (size_t g = 0; g < eqGroups_.size(); g++) {
            eqGroups_[g].findReferenceIteration();
#ifdef CPPADCG_PRINT_DEBUG
            std::cout << "reference iteration=" << eqGroups_[g].refIteration << "\n";
            print(eqGroups_[g].iterationDependents);
            std::cout << std::endl;

            typename std::set<EquationPattern<Base>*>::const_iterator iteq;
            for (iteq = eqGroups_[g].equations.begin(); iteq != eqGroups_[g].equations.end(); ++iteq) {
                std::cout << "eq dependents=";
                print((*iteq)->dependents);
                std::cout << std::endl;
            }
#endif
        }


        for (size_t g = 0; g < eqGroups_.size(); g++) {
            const EquationGroup<Base>& group = eqGroups_[g];
            const std::set<size_t>& refItDep = group.iterationDependents[group.refIteration];
            CPPADCG_ASSERT_UNKNOWN(refItDep.size() == group.equations.size())

            for (size_t dep : refItDep) {
                EquationPattern<Base>::uncolor(dependents[dep].getOperationNode(), *varIndexed_);
            }

            for (size_t dep : refItDep) {
                EquationPattern<Base>* eq = dep2Equation.at(dep);

                // operations that use indexed independent variables in the reference iteration
                std::set<const OperationNode<Base>*> indexedOperations;

                eq->findIndexedPath(dep, dependents, *varIndexed_, indexedOperations);
                if (dep == eq->depRefIndex) {
                    for (const auto& itop2a : eq->indexedOpIndep.op2Arguments) {
                        // currently there is no way to make a distinction between yi = xi and y_(i+1) = x_(i+1)
                        // since both operations which use indexed independents would be nullptr (the dependent)
                        // an alias is used for these cases
                        CPPADCG_ASSERT_UNKNOWN(itop2a.first != nullptr)
                        addOperationArguments2Loop(itop2a.first, itop2a.second);
                    }

                } else {
                    // generate loop references
                    for (const OperationNode<Base>* opLoopRef : indexedOperations) {
                        // currently there is no way to make a distinction between yi = xi and y_(i+1) = x_(i+1)
                        // since both operations which use indexed independents would be nullptr (the dependent)
                        // an alias is used for these cases
                        CPPADCG_ASSERT_UNKNOWN(opLoopRef != nullptr)

                        const OperationNode<Base>* opEqRef = eq->operationEO2Reference.at(dep).at(opLoopRef);
                        addOperationArguments2Loop(opLoopRef, eq->indexedOpIndep.op2Arguments.at(opEqRef));
                    }
                }

                // not needed anymore (lets free this memory now)
                eq->indexedOpIndep.op2Arguments.clear();
            }
        }

        /**
         * independent variable index patterns
         */
        generateIndependentLoopIndexes();

        if(varId_ != nullptr)
            varId_->fill(0);

        createLoopTapeNModel(dependents, independents, dep2Equation, origTemp2Index);

        /**
         * Clean-up
         */
        if(varId_ != nullptr)
            varId_->fill(0);
    }

    void generateDependentLoopIndexes(const std::map<size_t, EquationPattern<Base>*>& dep2Equation) {
        using namespace std;

        iterationDependents_.clear();
        equationOrder_.clear();
        iterationCount_ = 0;
        size_t nMaxIt = 0;
        /**
         * assign a dependent variable from each equation to an iteration
         */
        map<EquationPattern<Base>*, set<size_t> > depsInEq;

        for (EquationPattern<Base>* eq : equations) {
            depsInEq[eq] = eq->dependents;
            nMaxIt = std::max<size_t>(nMaxIt, eq->dependents.size());
        }

        iterationDependents_.reserve(nMaxIt + 2 * equations.size());

        for (size_t g = 0; g < eqGroups_.size(); g++) {
            EquationGroup<Base>& group = eqGroups_[g];
            const set<EquationPattern<Base>*>& eqs = group.equations;
            const std::vector<set<size_t> >& linkedDependents = group.linkedDependents;
            std::vector<set<size_t> >& relatedEqIterationDeps = group.iterationDependents;

            relatedEqIterationDeps.reserve(iterationDependents_.size());

            // assign an index to each equation
            for (EquationPattern<Base>* eq : eqs) {
                size_t eqo_size = equationOrder_.size();
                equationOrder_[eq] = eqo_size;
            }

            // sort dependents
            set<size_t> dependents;
            for (EquationPattern<Base>* eq : eqs) {
                dependents.insert(eq->dependents.begin(), eq->dependents.end());
            }

            map<size_t, map<EquationPattern<Base>*, set<size_t> > > nIndexedGroupPos2Eq2deps;


            map<EquationPattern<Base>*, std::vector<size_t> > freeDependents; // ordered dependents not assign to any iteration
            for (EquationPattern<Base>* eq : equations) {
                freeDependents[eq].assign(eq->dependents.begin(), eq->dependents.end());
            }

            DependentIndexSorter depSorter(group, freeDependents, dep2Equation);

            auto itDep = dependents.begin();
            size_t i = 0; // iteration counter
            while (itDep != dependents.end()) {
                size_t dep = *itDep;
                itDep++;

                if (iterationDependents_.size() <= i)
                    iterationDependents_.resize(i + 1);
                set<size_t>& itDepi = iterationDependents_[i];

                if (relatedEqIterationDeps.size() <= i)
                    relatedEqIterationDeps.resize(i + 1);
                set<size_t>& ritDepi = relatedEqIterationDeps[i];

                // this dependent might not be the best dependent if this equation is not present in all iterations

                EquationPattern<Base>* eq = dep2Equation.at(dep);
                auto bestDep = depSorter.findBestDependentForIteration(dep, eq);
                dep = bestDep.first;
                eq = bestDep.second;

                long pos = group.findIndexedLinkedDependent(dep);
                if (pos >= 0) {
                    // assign the dependent to the first iteration with all its relationships
                    for (size_t dep2 : linkedDependents[pos]) {
                        if (dep2 == *itDep) itDep++; //make sure the iterator is valid
                        itDepi.insert(dep2);
                        ritDepi.insert(dep2);
                        dependents.erase(dep2);

                        EquationPattern<Base>* eq2 = dep2Equation.at(dep2);
                        std::vector<size_t>& eq2FreeDep = freeDependents[eq2];
                        typename std::vector<size_t>::const_iterator itFreeDep2;
                        itFreeDep2 = std::find(eq2FreeDep.cbegin(), eq2FreeDep.cend(), dep2); // consider using lower_bound instead
                        CPPADCG_ASSERT_UNKNOWN(itFreeDep2 != eq2FreeDep.cend())

                        eq2FreeDep.erase(itFreeDep2);
                    }

                    i++;
                } else {
                    // maybe this dependent shares a non-indexed temporary variable
                    long posN = group.findNonIndexedLinkedRel(eq);
                    if (posN >= 0) {
                        // there are only non-indexed shared relations with other equations (delay processing...)
                        dependents.erase(dep); // safe because of itDep++
                        nIndexedGroupPos2Eq2deps[posN][eq].insert(dep);
                    } else {
                        itDepi.insert(dep);
                        ritDepi.insert(dep);

                        std::vector<size_t>& eqFreeDep = freeDependents[eq];
                        auto itFreeDep = find(eqFreeDep.begin(), eqFreeDep.end(), dep); // consider using lower_bound instead
                        CPPADCG_ASSERT_UNKNOWN(itFreeDep != eqFreeDep.end())

                        eqFreeDep.erase(itFreeDep);

                        i++;
                    }
                }

            }

            /**
             * place dependents which only share non-indexed variables
             */
            if (!nIndexedGroupPos2Eq2deps.empty()) {

                map<EquationPattern<Base>*, set<size_t> > eqIterations;
                for (size_t i = 0; i < relatedEqIterationDeps.size(); i++) {
                    const set<size_t>& deps = relatedEqIterationDeps[i];
                    for (size_t dep : deps) {
                        eqIterations[dep2Equation.at(dep)].insert(i);
                    }
                }

                for (auto& itPos2Eq2Dep : nIndexedGroupPos2Eq2deps) {
                    size_t posN = itPos2Eq2Dep.first;
                    // must pick one dependent from each equation for each iteration
                    std::vector<size_t> deps;
                    deps.reserve(itPos2Eq2Dep.second.size());

                    set<size_t> usedIterations; // iterations used by these equations
                    // determine used iteration indexes
                    const set<EquationPattern<Base>*>& relations = group.linkedEquationsByNonIndexedRel[posN];
                    for (EquationPattern<Base>* itRel : relations) {
                        const set<size_t>& iters = eqIterations[itRel];
                        usedIterations.insert(iters.begin(), iters.end());
                    }

                    while (true) {

                        // must pick one dependent from each equation for each iteration
                        deps.clear();

                        for (auto& itEq2Dep : itPos2Eq2Dep.second) {
                            if (!itEq2Dep.second.empty()) {
                                deps.push_back(*itEq2Dep.second.begin());
                                itEq2Dep.second.erase(itEq2Dep.second.begin());
                            }
                        }

                        if (deps.empty()) {
                            break; // done
                        }

                        // find a free iteration index
                        size_t i = 0;
                        set<size_t>::const_iterator itIter;
                        for (itIter = usedIterations.begin(); itIter != usedIterations.end();) {
                            size_t i1 = *itIter;
                            ++itIter;
                            if (itIter != usedIterations.end()) {
                                size_t i2 = *itIter;
                                if (i2 - i1 != 1) {
                                    i = i1 + 1;
                                    break;
                                }
                            } else {
                                i = i1 + 1;
                            }
                        }

                        // add the dependents to the iteration
                        usedIterations.insert(i);

                        if (iterationDependents_.size() <= i)
                            iterationDependents_.resize(i + 1);
                        set<size_t>& itDepi = iterationDependents_[i];

                        if (relatedEqIterationDeps.size() <= i)
                            relatedEqIterationDeps.resize(i + 1);
                        set<size_t>& ritDepi = relatedEqIterationDeps[i];
                        itDepi.insert(deps.begin(), deps.end());
                        ritDepi.insert(deps.begin(), deps.end());
                    }
                }
                /**
                 * @todo reorder iterations according to the lowest
                 *       dependent in each iteration if there were new
                 *       iterations only with dependents related by
                 *       non-indexed shared variables
                 */

            }

            iterationCount_ = std::max<size_t>(iterationCount_, iterationDependents_.size());
        }

    }

    /**
     * Destructor
     */
    virtual ~Loop() {
        for (const auto& itf : firstIndep2orders_) {
            for (IndependentOrder<Base>* ito : itf.second) {
                delete ito;
            }
        }

        for (OperationArgumentsIndepOrder<Base>* itio : arg2IndepOrder_) {
            delete itio;
        }

        for (EquationPattern<Base>* eq : equations) {
            delete eq;
        }

        delete loopModel_;
    }

    /***********************************************************************
     *                            private
     **********************************************************************/
private:

    void addOperationArguments2Loop(const OperationNode<Base>* op,
                                    const OperationIndexedIndependents<Base>& eqOpIndeIndep) {
        CPPADCG_ASSERT_UNKNOWN(!dep2Iteration_.empty());

        OperationIndexedIndependents<Base>& loopOpIndeIndep = indexedOpIndep.op2Arguments[op];
        loopOpIndeIndep.arg2Independents.resize(eqOpIndeIndep.arg2Independents.size());

        // some iterations might have not been defined by previous equation patterns
        for (size_t a = 0; a < eqOpIndeIndep.arg2Independents.size(); a++) {
            if (eqOpIndeIndep.arg2Independents[a].empty())
                continue;

            for (const auto& itDepIndep : eqOpIndeIndep.arg2Independents[a]) {
                size_t dep = itDepIndep.first;
                size_t iter = dep2Iteration_.at(dep);
                loopOpIndeIndep.arg2Independents[a][iter] = itDepIndep.second;
            }
        }

    }

    void generateIndependentLoopIndexes() {
        CPPADCG_ASSERT_UNKNOWN(iterationCount_ > 0); //number of iterations and dependent indexes must have already been determined

        // loop all operations from the reference dependents which use indexed independents
        for (const auto& it : indexedOpIndep.op2Arguments) {
            const OperationNode<Base>* operation = it.first;
            const OperationIndexedIndependents<Base>& opInd = it.second;

            auto* arg2orderPos = new OperationArgumentsIndepOrder<Base>();
            op2Arg2IndepOrder_[operation] = arg2orderPos;

            arg2IndepOrder_.push_front(arg2orderPos);

            // loop all arguments
            size_t aSize = opInd.arg2Independents.size();
            for (size_t argumentIndex = 0; argumentIndex < aSize; argumentIndex++) {
                const std::map<size_t, const OperationNode<Base>*>& dep2Indep = opInd.arg2Independents[argumentIndex];
                if (dep2Indep.empty())
                    continue; // not an indexed variable

                std::vector<const OperationNode<Base>*> order(iterationCount_);

                /**
                 * create the independent variable order
                 */
                CPPADCG_ASSERT_UNKNOWN(dep2Indep.size() > 0 && dep2Indep.size() <= iterationCount_)
                for (const auto& itDep2Indep : dep2Indep) {
                    size_t iterationIndex = itDep2Indep.first;
                    const OperationNode<Base>* indep = itDep2Indep.second;

                    order[iterationIndex] = indep;
                }

                /**
                 * try to find an existing independent variable order
                 */
                std::vector<IndependentOrder<Base>*>& availableOrders = firstIndep2orders_[order[0]];
                IndependentOrder<Base>* match = nullptr;

                long a_size = availableOrders.size();
                for (long o = 0; o < a_size; o++) {
                    IndependentOrder<Base>* orderO = availableOrders[o];
                    bool ok = true;
                    for (size_t iterationIndex = 0; iterationIndex < iterationCount_; iterationIndex++) {
                        if (orderO->order[iterationIndex] != order[iterationIndex]) {
                            ok = false;
                            break;
                        }
                    }
                    if (ok) {
                        match = orderO;
                        break;
                    }
                }

                if (match != nullptr) {
                    // found another operation with the same independent variable order
                    arg2orderPos->arg2Order[argumentIndex] = match;
                } else {
                    // brand new independent variable order
                    auto* iOrder = new IndependentOrder<Base>(order);
                    availableOrders.push_back(iOrder);
                    arg2orderPos->arg2Order[argumentIndex] = iOrder;
                }

            }

        }
    }

    /**
     * Creates the model for the loop equations
     *
     * @param dependents original model dependent variable vector
     * @param independents original model independent variable vector
     * @param dep2Equation maps an equation/dependent index to an equation pattern
     * @param origTemp2Index
     */
    void createLoopTapeNModel(const std::vector<CG<Base> >& dependents,
                              const std::vector<CG<Base> >& independents,
                              const std::map<size_t, EquationPattern<Base>*>& dep2Equation,
                              std::map<OperationNode<Base>*, size_t>& origTemp2Index) {
        using CGB = CG<Base>;
        using ADCGB = AD<CGB>;
        CPPADCG_ASSERT_UNKNOWN(independents.size() > 0)
        CPPADCG_ASSERT_UNKNOWN(independents[0].getCodeHandler() != nullptr)
        CodeHandler<Base>& origHandler = *independents[0].getCodeHandler();

        /**
         * create the new/clone operations for the reference iteration only
         */
        nIndependents_ = 0;
        idCounter_ = 1;

        CPPADCG_ASSERT_UNKNOWN(equationOrder_.size() == equations.size())

        std::vector<CGB> deps(equations.size());

        for (size_t g = 0; g < eqGroups_.size(); g++) {
            const EquationGroup<Base>& group = eqGroups_[g];
            const std::set<size_t>& iterationDependents = group.iterationDependents[group.refIteration];

            for (size_t depIndex : iterationDependents) {
                EquationPattern<Base>* eq = dep2Equation.at(depIndex);
                OperationNode<Base>* node = dependents[depIndex].getOperationNode();

                Argument<Base> aClone;

                if (node != nullptr) {
                    if (node->getOperationType() == CGOpCode::Inv) {
                        aClone = createIndependentClone(nullptr, 0, *node);
                    } else {
                        aClone = makeGraphClones(*eq, *node);
                    }
                } else {
                    aClone = dependents[depIndex].getValue();
                }

                size_t i = equationOrder_.at(eq);
                deps[i] = CGB(aClone);
            }
        }

        /*******************************************************************
         * create the tape for the reference iteration
         ******************************************************************/
        // indexed independents
        CPPADCG_ASSERT_UNKNOWN(indexedIndep2clone_.size() == independentsIndexed_.size());

        std::vector<const OperationNode<Base>*> indexedCloneOrder;
        indexedCloneOrder.reserve(indexedIndep2clone_.size());

        std::map<const OperationNode<Base>*, const IndependentOrder<Base>*> clone2indexedIndep;
        for (const auto& it : indexedIndep2clone_) {
            clone2indexedIndep[it.second] = it.first;
            indexedCloneOrder.push_back(it.second);
        }

        struct IndexedIndepSorter indexedSorter(clone2indexedIndep);
        std::sort(indexedCloneOrder.begin(), indexedCloneOrder.end(), indexedSorter);

        // original indep index -> non-indexed independent clones
        std::map<size_t, const OperationNode<Base>*> nonIndexedCloneOrder;

        // [tape variable] = original independent index
        std::map<const OperationNode<Base>*, const OperationNode<Base>*> clones2ConstIndep;
        size_t s = 0;
        for (auto itc = orig2ConstIndepClone_.begin(); itc != orig2ConstIndepClone_.end(); ++itc, s++) {
            const OperationNode<Base>* orig = itc->first;
            OperationNode<Base>* clone = itc->second;

            size_t j = orig->getInfo()[0];
            clones2ConstIndep[clone] = orig;
            nonIndexedCloneOrder[j] = clone;
        }

        size_t nIndexed = indexedCloneOrder.size();
        size_t nNonIndexed = nonIndexedCloneOrder.size();
        size_t nTmpIndexed = independentsTemp_.size();

        // tape independent array
        size_t nIndep = independentsIndexed_.size() +
                independentsNonIndexed_.size() +
                independentsTemp_.size();
        std::vector<ADCGB> loopIndeps(nIndep);

        typename std::map<const OperationNode<Base>*, IndexValue>::const_iterator itt;
        typename std::map<size_t, const OperationNode<Base>*>::const_iterator origJ2CloneIt;

        if (nIndep == 0) {
            loopIndeps.resize(1); // the tape cannot have 0 independents
            loopIndeps[0] = Base(0);

        } else {

            /**
             * assign values from the original model if possible (to avoid NaN)
             */
            for (size_t j = 0; j < nIndexed; j++) {
                const IndexValue& iv = independentsIndexed_.at(indexedCloneOrder[j]);
                loopIndeps[j] = iv.second;
            }

            s = 0;
            for (origJ2CloneIt = nonIndexedCloneOrder.begin(); origJ2CloneIt != nonIndexedCloneOrder.end(); ++origJ2CloneIt, s++) {
                const IndexValue& iv = independentsNonIndexed_.at(origJ2CloneIt->second);
                loopIndeps[nIndexed + s] = iv.second;
            }

            s = 0;
            for (itt = independentsTemp_.begin(); itt != independentsTemp_.end(); ++itt, s++) {
                const IndexValue& iv = itt->second;
                loopIndeps[nIndexed + nNonIndexed + s] = iv.second;
            }
        }

        /**
         * register the independent variables
         */
        CppAD::Independent(loopIndeps);

        /**
         * Reorder independent variables for the new tape (reference iteration only)
         */
        std::vector<ADCGB> localIndeps(nIndep);
        if (nIndep > 0) {
            for (size_t j = 0; j < nIndexed; j++) {
                const IndexValue& iv = independentsIndexed_.at(indexedCloneOrder[j]);
                size_t localIndex = iv.first;
                localIndeps[localIndex] = loopIndeps[j];
            }

            s = 0;
            for (origJ2CloneIt = nonIndexedCloneOrder.begin(); origJ2CloneIt != nonIndexedCloneOrder.end(); ++origJ2CloneIt, s++) {
                size_t localIndex = independentsNonIndexed_.at(origJ2CloneIt->second).first;
                localIndeps[localIndex] = loopIndeps[nIndexed + s];
            }

            s = 0;
            for (itt = independentsTemp_.begin(); itt != independentsTemp_.end(); ++itt, s++) {
                size_t localIndex = itt->second.first;
                localIndeps[localIndex] = loopIndeps[nIndexed + nNonIndexed + s];
            }
        }

        /**
         * create the tape
         */
        Evaluator<Base, CGB> evaluator1stIt(handler_);

        // load any atomic function used in the original model
        const std::map<size_t, CGAbstractAtomicFun<Base>* >& atomicsOrig = origHandler.getAtomicFunctions();
        std::map<size_t, atomic_base<CGB>* > atomics;
        atomics.insert(atomicsOrig.begin(), atomicsOrig.end());
        evaluator1stIt.addAtomicFunctions(atomics);

        std::vector<ADCGB> newDeps = evaluator1stIt.evaluate(localIndeps, deps);

        bool containsAtoms = evaluator1stIt.getNumberOfEvaluatedAtomics() > 0;

        std::unique_ptr<ADFun<CGB> >funIndexed(new ADFun<CGB>());
        funIndexed->Dependent(newDeps);

        /*******************************************************************
         * create the loop model object
         ******************************************************************/
        std::vector<std::vector<size_t> > dependentOrigIndexes(equations.size(), std::vector<size_t> (iterationCount_, (std::numeric_limits<size_t>::max)()));
        for (size_t it = 0; it < iterationCount_; it++) {
            const std::set<size_t>& itDeps = iterationDependents_[it];
            for (size_t origDep : itDeps) {
                EquationPattern<Base>* eq = dep2Equation.at(origDep);
                size_t i = equationOrder_.at(eq);
                dependentOrigIndexes[i][it] = origDep;
            }
        }

        //[tape variable][iteration] =  original independent index
        std::vector<std::vector<size_t> > indexedIndependents(nIndexed, std::vector<size_t>(iterationCount_));
        for (size_t j = 0; j < indexedCloneOrder.size(); j++) {
            const IndependentOrder<Base>* origOrder = clone2indexedIndep.at(indexedCloneOrder[j]);
            for (size_t it = 0; it < iterationCount_; it++) {
                const OperationNode<Base>* indep = origOrder->order[it];
                size_t index;
                if (indep != nullptr) {
                    index = indep->getInfo()[0];
                } else {
                    index = (std::numeric_limits<size_t>::max)(); // not used at this iteration by any equation
                }
                indexedIndependents[j][it] = index;
            }
        }

        std::vector<size_t> temporaryIndependents(nTmpIndexed);

        size_t j = 0;
        for (itt = independentsTemp_.begin(); itt != independentsTemp_.end(); ++itt, j++) {
            const OperationNode<Base>* tmpClone = itt->first;
            OperationNode<Base>* origTmpNode = temporaryClone2Orig_.at(tmpClone);

            /**
             * assign an index (k) to each temporary variable
             */
            size_t k;
            typename std::map<OperationNode<Base>*, size_t>::const_iterator itz = origTemp2Index.find(origTmpNode);
            if (itz == origTemp2Index.end()) {
                k = origTemp2Index.size();
                origTemp2Index[origTmpNode] = k; // new index for a temporary variable
            } else {
                k = itz->second;
            }

            temporaryIndependents[j] = k;
        }

        std::vector<size_t> nonIndexedIndependents(orig2ConstIndepClone_.size());
        s = 0;
        for (origJ2CloneIt = nonIndexedCloneOrder.begin(); origJ2CloneIt != nonIndexedCloneOrder.end(); ++origJ2CloneIt, s++) {
            nonIndexedIndependents[s] = origJ2CloneIt->first;
        }

        loopModel_ = new LoopModel<Base>(funIndexed.release(),
                                         containsAtoms,
                                         iterationCount_,
                                         dependentOrigIndexes,
                                         indexedIndependents,
                                         nonIndexedIndependents,
                                         temporaryIndependents);

        loopModel_->detectIndexPatterns();
    }

    inline Argument<Base> makeGraphClones(const EquationPattern<Base>& eq,
                                          OperationNode<Base>& node) {

        CPPADCG_ASSERT_UNKNOWN(node.getOperationType() != CGOpCode::Inv)

        size_t id = (*varId_)[node];

        if (id > 0) {
            // been here before
            return Argument<Base>(*clonesTemporary_.at(id));
        }

        //handler_.markVisited(node);

        id = idCounter_++;
        (*varId_)[node] = id;

        if ((*varIndexed_)[node] || node.getOperationType() == CGOpCode::ArrayCreation) {
            /**
             * part of the operation path that depends on the loop indexes
             * or its an array with constant elements
             */
            const std::vector<Argument<Base> >& args = node.getArguments();
            size_t arg_size = args.size();
            std::vector<Argument<Base> > cloneArgs(arg_size);

            for (size_t a = 0; a < arg_size; a++) {
                OperationNode<Base>* argOp = args[a].getOperation();
                if (argOp == nullptr) {
                    // parameter
                    cloneArgs[a] = *args[a].getParameter();
                } else {
                    // variable
                    if (argOp->getOperationType() == CGOpCode::Inv) {
                        cloneArgs[a] = createIndependentClone(&node, a, *argOp);
                    } else {
                        cloneArgs[a] = makeGraphClones(eq, *argOp);
                    }
                }
            }

            OperationNode<Base>* cloneOp = handler_.makeNode(node.getOperationType(),
                                                             node.getInfo(),
                                                             cloneArgs);

            clonesTemporary_[id] = cloneOp;
            return Argument<Base>(*cloneOp);

        } else {
            /**
             * temporary variable used in all iterations
             * (does not depend on indexes)
             */
            return makeTemporaryVarClone(node);
        }
    }

    inline OperationNode<Base>& createIndependentClone(OperationNode<Base>* operation,
                                                       size_t argumentIndex,
                                                       OperationNode<Base>& independent) {

        // is it an indexed independent?
        typename std::map<const OperationNode<Base>*, OperationIndexedIndependents<Base> >::const_iterator it = indexedOpIndep.op2Arguments.find(operation);
        if (it != indexedOpIndep.op2Arguments.end()) {
            const OperationIndexedIndependents<Base>& yyy = it->second;

            if (!yyy.arg2Independents[argumentIndex].empty()) {
                // yes
                return getIndexedIndependentClone(operation, argumentIndex);
            }
        }

        // it is constant for all operation
        return getNonIndexedIndependentClone(independent);
    }

    OperationNode<Base>& getIndexedIndependentClone(const OperationNode<Base>* operation,
                                                    size_t argIndex) {
        CPPADCG_ASSERT_UNKNOWN(operation == nullptr || operation->getArguments().size() > argIndex)
        CPPADCG_ASSERT_UNKNOWN(operation == nullptr || operation->getArguments()[argIndex].getOperation() != nullptr)
        CPPADCG_ASSERT_UNKNOWN(operation == nullptr || operation->getArguments()[argIndex].getOperation()->getOperationType() == CGOpCode::Inv)

        OperationArgumentsIndepOrder<Base>* args2Order = op2Arg2IndepOrder_.at(operation);
        IndependentOrder<Base>* indepOrder = args2Order->arg2Order.at(argIndex);

        typename std::map<const IndependentOrder<Base>*, OperationNode<Base>*>::const_iterator it;
        it = indexedIndep2clone_.find(indepOrder);
        if (it != indexedIndep2clone_.end()) {
            return *it->second;
        } else {
            CG<Base> newIndep;
            handler_.makeVariable(newIndep);
            independentsIndexed_[newIndep.getOperationNode()] = IndexValue(nIndependents_, newIndep);
            nIndependents_++;

            OperationNode<Base>* clone = newIndep.getOperationNode();
            indexedIndep2clone_[indepOrder] = clone;
            return *clone;
        }
    }

    OperationNode<Base>& getNonIndexedIndependentClone(const OperationNode<Base>& node) {
        CPPADCG_ASSERT_UNKNOWN(node.getOperationType() == CGOpCode::Inv)

        typename std::map<const OperationNode<Base>*, OperationNode<Base>*>::iterator it;
        it = orig2ConstIndepClone_.find(&node);
        if (it != orig2ConstIndepClone_.end()) {
            return *it->second;
        }

        CG<Base> newIndep;
        handler_.makeVariable(newIndep);
        independentsNonIndexed_[newIndep.getOperationNode()] = IndexValue(nIndependents_, newIndep);
        nIndependents_++;

        OperationNode<Base>* clone = newIndep.getOperationNode();
        orig2ConstIndepClone_[&node] = clone;
        return *clone;
    }

    /**
     * Creates a temporary variable that does NOT depend on the loop indexes
     *
     * @param node The original node
     * @return the clone
     */
    OperationNode<Base>& makeTemporaryVarClone(OperationNode<Base>& node) {
        CPPADCG_ASSERT_UNKNOWN(node.getOperationType() != CGOpCode::Inv)
        CPPADCG_ASSERT_UNKNOWN(node.getOperationType() != CGOpCode::ArrayCreation)
        CPPADCG_ASSERT_UNKNOWN(node.getOperationType() != CGOpCode::SparseArrayCreation)

        CG<Base> newIndep;
        handler_.makeVariable(newIndep);
        OperationNode<Base>* cloneOp = newIndep.getOperationNode();

        temporaryClone2Orig_[cloneOp] = &node;
        independentsTemp_[cloneOp] = IndexValue(nIndependents_, newIndep);
        nIndependents_++;

        size_t id = idCounter_++;
        clonesTemporary_[id] = cloneOp;
        (*varId_)[node] = id;

        return *cloneOp;
    }

    /**
     * Class used to help assign dependent variables to iterations
     */
    class DependentIndexSorter {
    private:
        const EquationGroup<Base>& group;
        const std::map<EquationPattern<Base>*, std::vector<size_t> >& freeDependents;
        const std::map<size_t, EquationPattern<Base>*>& dep2Equation;
        std::set<EquationPattern<Base>*> visitedEq;
    public:

        inline DependentIndexSorter(const EquationGroup<Base>& g,
                                    const std::map<EquationPattern<Base>*, std::vector<size_t> >& f,
                                    const std::map<size_t, EquationPattern<Base>*>& d2e) :
            group(g),
            freeDependents(f),
            dep2Equation(d2e) {
        }

        /**
         * Tries to find the best dependent to use for the next iteration.
         * For performance reasons it might not be the optimum.
         *
         * @param dep The dependent index
         * @param eq The equation
         * @return the best dependent index and its equation
         */
        inline std::pair<size_t, EquationPattern<Base>*> findBestDependentForIteration(size_t dep, EquationPattern<Base>* eq) {
            visitedEq.clear();
            return findBestDependent(dep, eq);
        }

    private:

        inline size_t findRelativeFreeDependentInEq(EquationPattern<Base>* eq, size_t dep) {
            // find relative order of the dependent in the equation
            const std::vector<size_t>& eqFreeDep = freeDependents.at(eq);
            for (size_t depRelPos = 0; depRelPos < eqFreeDep.size(); depRelPos++) {// consider using lower_bound
                if (eqFreeDep[depRelPos] == dep)
                    return depRelPos; // found it
            }

            assert(false);
            return eqFreeDep.size(); // should not happen
        }

        /**
         * Tries to find the best dependent to use for the next iteration
         *
         * @param dep The dependent index
         * @param eq The equation
         * @return the best dependent index and its equation
         */
        inline std::pair<size_t, EquationPattern<Base>*> findBestDependent(size_t dep, EquationPattern<Base>* eq) {
            visitedEq.insert(eq);

            auto best = std::make_pair(dep, eq);

            // find the group of dependents in the same iteration (associated by indexed independents)
            long pos = group.findIndexedLinkedDependent(dep);
            if (pos >= 0) {
                size_t depRelPos = findRelativeFreeDependentInEq(eq, dep);

                // this dependent might not be the best dependent if this
                // equation is not present in all iterations
                for (size_t dep2 : group.linkedDependents[pos]) {
                    EquationPattern<Base>* eq2 = dep2Equation.at(dep2);
                    if (visitedEq.find(eq2) != visitedEq.end()) continue;

                    size_t dep2RelPos = findRelativeFreeDependentInEq(eq2, dep2);
                    if (dep2RelPos > depRelPos) {
                        // this equation has more dependents before
                        // one these other dependents would be better
                        best = std::make_pair(dep2, eq2);
                        depRelPos = dep2RelPos;
                    }
                }

                if (best.first != dep) {
                    size_t bestDep = *freeDependents.at(best.second).begin();
                    return findBestDependent(bestDep, best.second);
                }

            }

            return best;
        }

    };

    /**
     * structure used to sort the loop's indexed independent variables
     */
    struct IndexedIndepSorter {
        const std::map<const OperationNode<Base>*, const IndependentOrder<Base>*>& clone2indexedIndep;

        IndexedIndepSorter(const std::map<const OperationNode<Base>*, const IndependentOrder<Base>*>& clone2indexedIndep_) :
            clone2indexedIndep(clone2indexedIndep_) {
        }

        bool operator()(const OperationNode<Base>* node1,
                const OperationNode<Base>* node2) {
            const IndependentOrder<Base>* indepOrder1 = clone2indexedIndep.at(node1);
            const IndependentOrder<Base>* indepOrder2 = clone2indexedIndep.at(node2);
            CPPADCG_ASSERT_UNKNOWN(indepOrder1->order.size() == indepOrder2->order.size())

            size_t size = indepOrder1->order.size();
            for (size_t j = 0; j < size; j++) {
                const OperationNode<Base>* indep1 = indepOrder1->order[j];
                const OperationNode<Base>* indep2 = indepOrder2->order[j];
                // some variables are not used in all iterations
                if (indep1 == nullptr) {
                    if (indep2 == nullptr) {
                        continue;
                    }
                    return false;
                } else if (indep2 == nullptr) {
                    return true;
                }

                size_t index1 = indep1->getInfo()[0];
                size_t index2 = indep2->getInfo()[0];
                if (index1 < index2)
                    return true;
                else if (index1 > index2)
                    return false;
            }

            CPPADCG_ASSERT_UNKNOWN(false) // should never get here
            return false;
        }
    };

};

} // END cg namespace
} // END CppAD namespace

#endif
