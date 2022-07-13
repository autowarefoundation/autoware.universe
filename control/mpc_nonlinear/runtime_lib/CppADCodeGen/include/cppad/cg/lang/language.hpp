#ifndef CPPAD_CG_LANGUAGE_INCLUDED
#define CPPAD_CG_LANGUAGE_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

/**
 * Information required for the generation of source code for a language
 *
 * @author Joao Leal
 */
template<class Base>
class LanguageGenerationData {
public:
    using Node = OperationNode<Base>;
    using ScopeIDType = typename CodeHandler<Base>::ScopeIDType;
public:
    /**
     * The independent variables
     */
    const std::vector<Node *>& independent;
    /**
     * The dependent variables
     */
    const ArrayView<CG<Base> >& dependent;
    /**
     * The lowest ID used for temporary variables
     */
    size_t minTemporaryVarID;
    /**
     * Provides the variable ID that was altered/assigned to operation nodes.
     * Zero means that no variable is assigned.
     */
    const CodeHandlerVector<Base, size_t>& varId;
    /**
     * Variable assignment order in the source code
     */
    const std::vector<Node*>& variableOrder;
    /**
     * maps dependencies between variables in variableOrder
     */
    const std::vector<std::set<Node*>>& variableDependencies;
    /**
     * Provides the rules for variable name creation
     */
    VariableNameGenerator<Base>& nameGen;
    /**
     * maps atomic function IDs to their internal index
     */
    const std::map<size_t, size_t>& atomicFunctionId2Index;
    /**
     * maps atomic function IDs to their names
     */
    const std::map<size_t, std::string>& atomicFunctionId2Name;
    /**
     * the maximum forward mode order each atomic function is called
     * (-1 means forward mode not used)
     */
    const std::vector<int>& atomicFunctionsMaxForward;
    /**
     * the maximum reverse mode order each atomic function is called
     * (-1 means reverse mode not used)
     */
    const std::vector<int>& atomicFunctionsMaxReverse;
    /**
     * a flag indicating whether or not temporary variable IDs have been recycled
     */
    const bool reuseIDs;
    //
    const std::set<const Node*>& indexes;
    //
    const std::set<RandomIndexPattern*>& indexRandomPatterns;
    //
    const std::vector<IndexPattern*>& loopDependentIndexPatterns;
    //
    const std::vector<IndexPattern*>& loopIndependentIndexPatterns;
    /**
     * the total number of times the result of an operation node  is used
     */
    const CodeHandlerVector<Base, size_t>& totalUseCount;
    /**
     * scope of each managed operation node
     */
    const CodeHandlerVector<Base, ScopeIDType>& scope;
    /**
     * Auxiliary index (might not be used)
     */
    IndexOperationNode<Base>& auxIterationIndexOp;
    /**
     * whether or not the dependent variables should be zeroed before
     * executing the operation graph
     */
    const bool zeroDependents;
public:

    LanguageGenerationData(const std::vector<Node *>& ind,
                           const ArrayView<CG<Base> >& dep,
                           size_t minTempVID,
                           const CodeHandlerVector<Base, size_t>& varIds,
                           const std::vector<Node*>& vo,
                           const std::vector<std::set<Node*>>& variableDependencies,
                           VariableNameGenerator<Base>& ng,
                           const std::map<size_t, size_t>& atomicId2Index,
                           const std::map<size_t, std::string>& atomicId2Name,
                           const std::vector<int>& atomicMaxForward,
                           const std::vector<int>& atomicMaxReverse,
                           const bool ri,
                           const std::set<const Node*>& indexes,
                           const std::set<RandomIndexPattern*>& idxRandomPatterns,
                           const std::vector<IndexPattern*>& dependentIndexPatterns,
                           const std::vector<IndexPattern*>& independentIndexPatterns,
                           const CodeHandlerVector<Base, size_t>& totalUseCount,
                           const CodeHandlerVector<Base, ScopeIDType>& scope,
                           IndexOperationNode<Base>& auxIterationIndexOp,
                           bool zero) :
        independent(ind),
        dependent(dep),
        minTemporaryVarID(minTempVID),
        varId(varIds),
        variableOrder(vo),
        variableDependencies(variableDependencies),
        nameGen(ng),
        atomicFunctionId2Index(atomicId2Index),
        atomicFunctionId2Name(atomicId2Name),
        atomicFunctionsMaxForward(atomicMaxForward),
        atomicFunctionsMaxReverse(atomicMaxReverse),
        reuseIDs(ri),
        indexes(indexes),
        indexRandomPatterns(idxRandomPatterns),
        loopDependentIndexPatterns(dependentIndexPatterns),
        loopIndependentIndexPatterns(independentIndexPatterns),
        totalUseCount(totalUseCount),
        scope(scope),
        auxIterationIndexOp(auxIterationIndexOp),
        zeroDependents(zero) {
    }
};

/**
 * Creates the source code for a specific language
 *
 * @author Joao Leal
 */
template<class Base>
class Language {
    friend class CodeHandler<Base>;
public:
    using Node = OperationNode<Base>;
protected:
    virtual void generateSourceCode(std::ostream& out,
                                    std::unique_ptr<LanguageGenerationData<Base> > info) = 0;

    /**
     * Whether or not a new variable is created as a result of this operation
     *
     * @param op Operation
     * @param totalUseCount the number of times this node has been used
     *                      as an argument in other operations
     * @return true if a new variable is created
     */
    virtual bool createsNewVariable(const Node& op,
                                    size_t totalUseCount,
                                    size_t opCount) const = 0;

    virtual bool requiresVariableArgument(enum CGOpCode op,
                                          size_t argIndex) const = 0;

    /**
     * Whether or not this language can use information regarding the
     * dependencies between different equations/variables.
     */
    virtual bool requiresVariableDependencies() const = 0;

};

} // END cg namespace
} // END CppAD namespace

#endif
