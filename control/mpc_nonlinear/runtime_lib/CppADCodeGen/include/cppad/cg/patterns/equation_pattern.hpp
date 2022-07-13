#ifndef CPPAD_CG_EQUATION_PATTERN_INCLUDED
#define CPPAD_CG_EQUATION_PATTERN_INCLUDED
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
 * Holds information on which independents are used by which
 * dependent in an equation pattern
 */
template<class Base>
class OperationIndexedIndependents {
public:
    using MapDep2Indep_type = std::map<size_t, const OperationNode<Base>*>;
    /**
     * maps the argument index to the several independents used in different
     * equations with the same pattern
     * argument index -> (iteration or dependent using it) -> independent
     */
    std::vector<MapDep2Indep_type> arg2Independents;
};

template<class Base>
class IndexedIndependent {
public:
    std::map<const OperationNode<Base>*, OperationIndexedIndependents<Base> > op2Arguments;
public:

    const OperationIndexedIndependents<Base>& arguments(const OperationNode<Base>* operation) const {
        return op2Arguments.at(operation);
    }

    bool isIndexedOperationArgument(const OperationNode<Base>* node, size_t argIndex) const {
        const auto itIndexes = op2Arguments.find(node);
        if (itIndexes == op2Arguments.end()) {
            return false;
        }
        const OperationIndexedIndependents<Base>& indexedArgs = itIndexes->second;
        return indexedArgs.arg2Independents.size() > argIndex && !indexedArgs.arg2Independents[argIndex].empty();
    }

};

/**
 * Group of variables with the same evaluation pattern
 * (same equation different variables)
 */
template<class Base>
class EquationPattern {
public:
    const CG<Base>& depRef; // dependent reference
    const size_t depRefIndex;
    std::set<size_t> dependents;
    /**
     * maps node ID used by all dependents to the operations of the
     * reference dependent
     * [dependent index][op] = reference operation
     */
    std::map<size_t, std::map<const OperationNode<Base>*, OperationNode<Base>*> > operationEO2Reference;
    // std::map<size_t, std::vector<OperationNode<Base>*> > operationEO2Reference;
    /**
     * Maps the operations that used an indexed independents as direct
     * arguments
     * (reference operation -> argument indexes -> dependent <-> independents)
     */
    IndexedIndependent<Base> indexedOpIndep;
    /**
     * reference operation -> non indexed argument positions
     */
    std::map<const OperationNode<Base>*, std::set<size_t> > constOperationIndependents;

private:
    CodeHandler<Base>* const handler_;
    size_t currDep_;
    size_t minColor_;
    size_t cmpColor_;
public:

    explicit EquationPattern(const CG<Base>& ref,
                             size_t iDepRef) :
        depRef(ref),
        depRefIndex(iDepRef),
        dependents {iDepRef},
        handler_(ref.getCodeHandler()) {
    }

    EquationPattern(const EquationPattern<Base>& other) = delete;

    EquationPattern& operator=(const EquationPattern<Base>& rhs) = delete;

    bool testAdd(size_t iDep2,
                 const CG<Base>& dep2,
                 size_t& minColor,
                 CodeHandlerVector<Base, size_t>& varColor) {
        IndexedIndependent<Base> independentsBackup = indexedOpIndep;
        std::map<const OperationNode<Base>*, std::set<size_t> > constOperationIndependentsBackup = constOperationIndependents;
        std::map<size_t, std::map<const OperationNode<Base>*, OperationNode<Base>*> > operation2ReferenceBackup = operationEO2Reference;

        currDep_ = iDep2;
        minColor_ = minColor;
        cmpColor_ = minColor_;

        bool equals = comparePath(depRef, dep2, iDep2, varColor);

        minColor = cmpColor_;

        if (equals) {
            dependents.insert(iDep2);

            return true; // matches the reference pattern
        } else {
            // restore
            indexedOpIndep.op2Arguments.swap(independentsBackup.op2Arguments);
            constOperationIndependents.swap(constOperationIndependentsBackup);
            operationEO2Reference.swap(operation2ReferenceBackup);

            return false; // cannot be added
        }
    }

    inline void findIndexedPath(size_t dep,
                                const std::vector<CG<Base> >& depVals,
                                CodeHandlerVector<Base, bool>& varIndexed,
                                std::set<const OperationNode<Base>*>& indexedOperations) {
        findIndexedPath(depRef, depVals[dep], varIndexed, indexedOperations);
    }

    std::set<const OperationNode<Base>*> findOperationsUsingIndependents(OperationNode<Base>& node) const {
        std::set<const OperationNode<Base>*> ops;

        handler_->startNewOperationTreeVisit();

        findOperationsWithIndeps(node, ops);

        return ops;
    }

    static inline void uncolor(OperationNode<Base>* node,
                               CodeHandlerVector<Base, bool>& varIndexed) {
        if (node == nullptr || !varIndexed[*node])
            return;

        varIndexed[*node] = false;

        const std::vector<Argument<Base> >& args = node->getArguments();
        size_t size = args.size();
        for (size_t a = 0; a < size; a++) {
            uncolor(args[a].getOperation(), varIndexed);
        }
    }

    inline bool containsConstantIndependent(const OperationNode<Base>* operation, size_t argumentIndex) const {
        const auto it = constOperationIndependents.find(operation);
        if (it != constOperationIndependents.end()) {
            if (it->second.find(argumentIndex) != it->second.end()) {
                return true;
            }
        }
        return false;
    }

    /**
     * Determine which independent variables in the loop model do not
     * require an index (always the same for all iterations)
     */
    inline void detectNonIndexedIndependents() {
        using MapIndep2Dep_type = typename OperationIndexedIndependents<Base>::MapDep2Indep_type;

        // loop operations using independents
        auto itop2a = indexedOpIndep.op2Arguments.begin();
        while (itop2a != indexedOpIndep.op2Arguments.end()) {
            const OperationNode<Base>* parentOp = itop2a->first;
            OperationIndexedIndependents<Base>& arg2It = itop2a->second;

            bool emptyOp = true;

            // loop the arguments
            size_t aSize = arg2It.arg2Independents.size();
            for (size_t argIndex = 0; argIndex < aSize; argIndex++) {
                MapIndep2Dep_type& dep2Ind = arg2It.arg2Independents[argIndex];

                if (dep2Ind.empty())
                    continue; // argument does not use independents

                // loop dependents (iterations)
                bool isIndexed = false;
                typename MapIndep2Dep_type::const_iterator itDep2Ind = dep2Ind.begin();
                const OperationNode<Base>* indep = itDep2Ind->second;

                for (++itDep2Ind; itDep2Ind != dep2Ind.end(); ++itDep2Ind) {
                    if (indep != itDep2Ind->second) {
                        isIndexed = true;
                        break;
                    }
                }

                if (!isIndexed) {
                    // make it a non indexed independent
                    constOperationIndependents[parentOp].insert(argIndex);

                    // remove it from the indexed independents
                    dep2Ind.clear();
                } else {
                    emptyOp = false;
                }

            }

            if (emptyOp) {
                indexedOpIndep.op2Arguments.erase(itop2a++);
            } else {
                ++itop2a;
            }

        }

    }

    virtual ~EquationPattern() = default;

private:

    bool comparePath(const CG<Base>& dep1,
                     const CG<Base>& dep2,
                     size_t dep2Index,
                     CodeHandlerVector<Base, size_t>& varColor) {
        CodeHandler<Base>* h1 = dep1.getCodeHandler();
        CodeHandler<Base>* h2 = dep2.getCodeHandler();

        if (h1 != h2) {
            if (h1 != nullptr && h2 != nullptr)
                throw CGException("Only one code handler allowed");
            return false;
        }

        if (dep1.isParameter() && dep2.isParameter()) {
            return dep1.getValue() == dep2.getValue();

        } else if (dep1.isVariable() && dep2.isVariable()) {
            OperationNode<Base>* depRefOp = dep1.getOperationNode();
            OperationNode<Base>* dep2Op = dep2.getOperationNode();
            CPPADCG_ASSERT_UNKNOWN(depRefOp->getOperationType() != CGOpCode::Inv)

            return comparePath(depRefOp, dep2Op, dep2Index, varColor);
        }

        return false;
    }

    bool comparePath(OperationNode<Base>* scRef,
                     OperationNode<Base>* sc2,
                     size_t dep2,
                     CodeHandlerVector<Base, size_t>& varColor) {
        saveOperationReference(dep2, sc2, scRef);
        if (dependents.size() == 1) {
            saveOperationReference(depRefIndex, scRef, scRef);
        }

        while (scRef->getOperationType() == CGOpCode::Alias) {
            CPPADCG_ASSERT_KNOWN(scRef->getArguments().size() == 1, "Invalid number of arguments for alias")
            OperationNode<Base>* sc = scRef->getArguments()[0].getOperation();
            if (sc != nullptr && sc->getOperationType() == CGOpCode::Inv) break;  // an alias is used to distinguish between indexed dependents and indexed independents
            scRef = sc;
        }
        while (sc2->getOperationType() == CGOpCode::Alias) {
            CPPADCG_ASSERT_KNOWN(sc2->getArguments().size() == 1, "Invalid number of arguments for alias")
            OperationNode<Base>* sc = sc2->getArguments()[0].getOperation();
            if (sc != nullptr && sc->getOperationType() == CGOpCode::Inv) break;  // an alias is used to distinguish between indexed dependents and indexed independents
            sc2 = sc;
        }

        // check if these nodes where visited before
        if (varColor[*sc2] >= minColor_ && varColor[*scRef] >= minColor_) {
            /**
             * been here before for both nodes
             *  warning: if one would return varColor[*sc2] == varColor[*scRef]
             *  it could fail to detect some patterns! e.g.:
             *    it ref ->  v1 + v1 + v2
             *    it 2   ->  v3 + v1 + v1
             *   where v1, v2, v3 have the same expression pattern but
             *   correspond to different nodes
             */
            if (varColor[*sc2] == varColor[*scRef])
                return true;
        }
        varColor[*scRef] = cmpColor_;
        varColor[*sc2] = cmpColor_;
        cmpColor_++;


        if (scRef->getOperationType() != sc2->getOperationType()) {
            return false;
        }

        CPPADCG_ASSERT_UNKNOWN(scRef->getOperationType() != CGOpCode::Inv)

        const std::vector<size_t>& info1 = scRef->getInfo();
        const std::vector<size_t>& info2 = sc2->getInfo();
        if (info1.size() != info2.size()) {
            return false;
        }

        for (size_t e = 0; e < info1.size(); e++) {
            if (info1[e] != info2[e]) {
                return false;
            }
        }

        const std::vector<Argument<Base> >& args1 = scRef->getArguments();
        const std::vector<Argument<Base> >& args2 = sc2->getArguments();
        size_t size = args1.size();
        if (size != args2.size()) {
            return false;
        }
        for (size_t a = 0; a < size; a++) {
            const Argument<Base>& a1 = args1[a];
            const Argument<Base>& a2 = args2[a];

            if (a1.getParameter() != nullptr) {
                if (a2.getParameter() == nullptr || *a1.getParameter() != *a2.getParameter())
                    return false;
            } else {
                if (a2.getOperation() == nullptr) {
                    return false;
                }
                OperationNode<Base>* argRefOp = a1.getOperation();
                OperationNode<Base>* arg2Op = a2.getOperation();
                bool related;
                if (argRefOp->getOperationType() == CGOpCode::Inv) {
                    related = saveIndependent(scRef, a, argRefOp, arg2Op);
                } else {
                    related = comparePath(argRefOp, arg2Op, dep2, varColor);
                }

                if (!related)
                    return false;
            }
        }

        return true; // same pattern
    }

    inline void saveOperationReference(size_t dep2,
                                       const OperationNode<Base>* sc2,
                                       OperationNode<Base>* scRef) {
        operationEO2Reference[dep2][sc2] = scRef;
    }

    bool saveIndependent(const OperationNode<Base>* parentOp,
                         size_t argIndex,
                         const OperationNode<Base>* argRefOp,
                         const OperationNode<Base>* arg2Op) {
        if (argRefOp->getOperationType() != CGOpCode::Inv || arg2Op->getOperationType() != CGOpCode::Inv) {
            return false;
        }

        /**
         * Must consider that the independent might change from iteration to
         * iteration (even if now it won't)
         */
        const auto it = constOperationIndependents.find(parentOp);
        if (it != constOperationIndependents.end()) {
            if (it->second.find(argIndex) != it->second.end()) {
                return false;
            }
        }

        OperationIndexedIndependents<Base>& opIndexedIndep = indexedOpIndep.op2Arguments[parentOp];
        opIndexedIndep.arg2Independents.resize(parentOp != nullptr ? parentOp->getArguments().size() : 1);

        std::map<size_t, const OperationNode<Base>*>& dep2Indeps = opIndexedIndep.arg2Independents[argIndex];
        if (dep2Indeps.empty())
            dep2Indeps[depRefIndex] = argRefOp;
        dep2Indeps[currDep_] = arg2Op;

        return true; // same pattern
    }

    inline void findIndexedPath(const CG<Base>& depRef,
                                const CG<Base>& dep2,
                                CodeHandlerVector<Base, bool>& varIndexed,
                                std::set<const OperationNode<Base>*>& indexedOperations) {
        if (depRef.isVariable() && dep2.isVariable()) {
            OperationNode<Base>* depRefOp = depRef.getOperationNode();
            OperationNode<Base>* dep2Op = dep2.getOperationNode();
            if (depRefOp->getOperationType() != CGOpCode::Inv) {
                findIndexedPath(depRefOp, dep2Op, varIndexed, indexedOperations);
            } else {

                typename std::map<const OperationNode<Base>*, OperationIndexedIndependents<Base> >::iterator itop2a;
                itop2a = indexedOpIndep.op2Arguments.find(nullptr);
                if (itop2a != indexedOpIndep.op2Arguments.end() && !itop2a->second.arg2Independents[0].empty()) {
                    // depends on an index
                    indexedOperations.insert(nullptr);
                }
            }
        }
    }

    inline bool findIndexedPath(const OperationNode<Base>* scRef,
                                OperationNode<Base>* sc2,
                                CodeHandlerVector<Base, bool>& varIndexed,
                                std::set<const OperationNode<Base>*>& indexedOperations) {

        while (scRef->getOperationType() == CGOpCode::Alias) {
            CPPADCG_ASSERT_KNOWN(scRef->getArguments().size() == 1, "Invalid number of arguments for alias")
            OperationNode<Base>* sc = scRef->getArguments()[0].getOperation();
            if (sc != nullptr && sc->getOperationType() == CGOpCode::Inv) break; // an alias is used to distinguish between indexed dependents and indexed independents
            scRef = sc;
        }
        while (sc2->getOperationType() == CGOpCode::Alias) {
            CPPADCG_ASSERT_KNOWN(sc2->getArguments().size() == 1, "Invalid number of arguments for alias")
            OperationNode<Base>* sc = sc2->getArguments()[0].getOperation();
            if (sc != nullptr && sc->getOperationType() == CGOpCode::Inv) break; // an alias is used to distinguish between indexed dependents and indexed independents
            sc2 = sc;
        }

        CPPADCG_ASSERT_UNKNOWN(scRef->getOperationType() == sc2->getOperationType())

        const std::vector<Argument<Base> >& argsRef = scRef->getArguments();

        typename std::map<const OperationNode<Base>*, OperationIndexedIndependents<Base> >::iterator itop2a;
        bool searched = false;
        bool indexedDependentPath = false;
        bool usesIndexedIndependent = false; // directly uses an indexed independent

        size_t size = argsRef.size();
        for (size_t a = 0; a < size; a++) {
            OperationNode<Base>* argRefOp = argsRef[a].getOperation();
            if (argRefOp != nullptr) {
                bool indexedArg = false;
                if (argRefOp->getOperationType() == CGOpCode::Inv) {
                    // same independent variable can be used in multiple iterations
                    if (!searched) {
                        itop2a = indexedOpIndep.op2Arguments.find(scRef);
                        searched = true;
                    }
                    if (itop2a != indexedOpIndep.op2Arguments.end() && !itop2a->second.arg2Independents[a].empty()) {
                        // depends on an index
                        indexedArg = true;
                        indexedDependentPath = true;
                        usesIndexedIndependent = true;
                    }
                }

                if (!indexedArg) {
                    const std::vector<Argument<Base> >& args2 = sc2->getArguments();
                    CPPADCG_ASSERT_UNKNOWN(size == args2.size())
                    indexedDependentPath |= findIndexedPath(argsRef[a].getOperation(), args2[a].getOperation(), varIndexed, indexedOperations);
                }
            }
        }

        varIndexed[*sc2] = indexedDependentPath;

        if (usesIndexedIndependent)
            indexedOperations.insert(sc2);

        return indexedDependentPath;
    }

    void findOperationsWithIndeps(OperationNode<Base>& node,
                                  std::set<const OperationNode<Base>*>& ops) const {
        if (handler_->isVisited(node))
            return; // been here before

        handler_->markVisited(node);

        const std::vector<Argument<Base> >& args = node.getArguments();
        size_t size = args.size();
        for (size_t a = 0; a < size; a++) {
            OperationNode<Base>* argOp = args[a].getOperation();
            if (argOp != nullptr) {
                if (argOp->getOperationType() == CGOpCode::Inv) {
                    ops.insert(&node);
                } else {
                    findOperationsWithIndeps(*argOp, ops);
                }
            }
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
