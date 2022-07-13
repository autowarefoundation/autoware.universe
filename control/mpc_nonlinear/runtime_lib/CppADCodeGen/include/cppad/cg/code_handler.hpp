#ifndef CPPAD_CG_CODE_HANDLER_INCLUDED
#define CPPAD_CG_CODE_HANDLER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
 * Helper class to analyze the operation graph and generate source code
 * for several languages
 *
 * @author Joao Leal
 */
template<class Base>
class CodeHandler {
    friend class CodeHandlerVectorSync<Base>;
public:
    using PathNode = OperationPathNode<Base>;
    using SourceCodePath = std::vector<PathNode>;
    using ScopePath = std::vector<ScopePathElement<Base> >;
    using Node = OperationNode<Base>;
    using Arg = Argument<Base>;
    using CGB = CG<Base>;
    using ScopeIDType = unsigned short;
protected:
    struct LoopData; // forward declaration

protected:
    // counter used to determine visitation IDs for the operation tree
    size_t _idVisit;
    // counter used to generate variable IDs
    size_t _idCount;
    // counter used to generate array variable IDs
    size_t _idArrayCount;
    // counter used to generate sparse array variable IDs
    size_t _idSparseArrayCount;
    // counter used to generate IDs for atomic functions
    size_t _idAtomicCount;
    // the independent variables
    std::vector<Node *> _independentVariables;
    // the current dependent variables
    ArrayView<CGB>* _dependents;
    /**
     * nodes managed by this code handler which include all
     * all OperationNodes created by CG<Base> objects
     */
    std::vector<Node*> _codeBlocks;
    /**
     * All CodeHandlerVector associated with this code handler
     */
    std::set<CodeHandlerVectorSync<Base>*> _managedVectors;
    /**
     * the ID of the last visit to each managed node
     */
    CodeHandlerVector<Base, size_t> _lastVisit;
    /**
     * scope of each managed operation node
     */
    CodeHandlerVector<Base, ScopeIDType> _scope;
    /**
     * evaluation order of each managed node
     * (zero means that an evaluation position was never assigned)
     */
    CodeHandlerVector<Base, size_t> _evaluationOrder;
    /**
     * the last index in the evaluation order for which an operation node
     * is taken as an argument of another operation node.
     * (zero means that the node was never used)
     */
    CodeHandlerVector<Base, size_t> _lastUsageOrder;
    /**
     * the total number of times the result of an operation node  is used
     */
    CodeHandlerVector<Base, size_t> _totalUseCount;
    /**
     * The number of operations used in the expression that directly assign the value
     * to a (temporary/dependent) variable.
     * Operations used to compute other temporary variables are not considered.
     */
    CodeHandlerVector<Base, size_t> _operationCount;
    /**
     * Provides the variable ID that was altered/assigned to operation nodes.
     * Zero means that no variable is assigned.
     */
    CodeHandlerVector<Base, size_t> _varId;
    /**
     * the order for the variable creation in the source code
     */
    std::vector<Node*> _variableOrder;
    /**
     * maps dependencies between variables in _variableOrder
     */
    std::vector<std::set<Node*>> _variableDependencies;
    /**
     * the order for the variable creation in the source code
     * (each level represents a different variable scope)
     */
    std::vector<std::vector<Node*> > _scopedVariableOrder;
    /**
     *
     */
    LoopData _loops;
    /**
     * maps the IDs of the atomic functions
     */
    std::map<size_t, CGAbstractAtomicFun<Base>*> _atomicFunctions;
    /**
     * already used atomic function names (may contain names which were
     * used by previous calls to this/other CondeHandlers)
     */
    std::map<std::string, size_t> _atomicFunctionName2Index;
    /**
     * the order of the atomic functions (may contain names which were
     * used by previous calls to this/other CondeHandlers)
     */
    std::vector<std::string>* _atomicFunctionsOrder;
    /**
     * the maximum forward mode order each atomic function is called
     * (-1 means forward mode not used)
     */
    std::vector<int> _atomicFunctionsMaxForward;
    /**
     * the maximum reverse mode order each atomic function is called
     * (-1 means reverse mode not used)
     */
    std::vector<int> _atomicFunctionsMaxReverse;
    // a flag indicating if this handler was previously used to generate code
    bool _used;
    // a flag indicating whether or not to reuse the IDs of destroyed variables
    bool _reuseIDs;
    // scope color/index counter
    ScopeIDType _scopeColorCount;
    // the current scope color/index counter
    ScopeIDType _currentScopeColor;
    // all scopes
    std::vector<ScopePath> _scopes;
    // possible altered nodes due to scope conditionals (altered node <-> clone of original)
    std::list<std::pair<Node*, Node* > > _alteredNodes;
    // the language used for source code generation
    Language<Base>* _lang;
    // the lowest ID used for temporary variables
    size_t _minTemporaryVarID;
    /**
     * whether or not the dependent variables should be zeroed before
     * executing the operation graph
     */
    bool _zeroDependents;
    //
    bool _verbose;
    /**
     * used to track evaluation times and print out messages
     */
    JobTimer* _jobTimer;
    /**
     * Auxiliary index declaration (might not be used)
     */
    Node* _auxIndexI;
    /**
     * Auxiliary index (might not be used)
     */
    IndexOperationNode<Base>* _auxIterationIndexOp;
public:

    CodeHandler(size_t varCount = 50);

    CodeHandler(const CodeHandler&) = delete;

    CodeHandler& operator=(const CodeHandler&) = delete;

    /**
     * Destructor
     */
    inline virtual ~CodeHandler();

    /**
     * Defines whether or not to reuse the node IDs once they are not need by
     * a node anymore.
     */
    inline void setReuseVariableIDs(bool reuse);

    /**
     * Whether or not node IDs are reused once they are not need by a node
     * anymore.
     */
    inline bool isReuseVariableIDs() const;

    /**
     * Marks the provided variables as being independent variables.
     *
     * @param variables the vector of variables that will become independent
     *                  variables.
     */
    template<class VectorCG>
    inline void makeVariables(VectorCG& variables) {
        for (size_t i = 0; i < variables.size(); i++) {
            makeVariable(variables[i]);
        }
    }

    /**
     * Marks the provided variables as being independent variables.
     *
     * @param variables the vector of variables that will become independent
     *                  variables.
     */
    inline void makeVariables(std::vector<AD<CGB> >& variables);

    /**
     * Marks the provided variable as being an independent variable.
     *
     * @param variables the variable that will become an independent
     *                  variables.
     */
    inline void makeVariable(AD<CGB>& variable);

    /**
     * Marks the provided variable as being an independent variable.
     *
     * @param variables the variable that will become an independent
     *                  variables.
     */
    inline void makeVariable(CGB& variable);

    /**
     * The number of independent variables defined with makeVariable().
     */
    size_t getIndependentVariableSize() const;

    /**
     * @throws CGException if a variable is not found in the independent vector
     */
    size_t getIndependentVariableIndex(const Node& var) const;

    /**
     * Provides variable IDs that were assigned to operation nodes.
     * Zero means that no variable is assigned.
     * The first IDs are reserved for the independent variables.
     * It can be an empty vector if IDs have not yet been assigned.
     */
    inline const CodeHandlerVector<Base, size_t>& getVariablesIDs() const;

    inline size_t getMaximumVariableID() const;

    inline bool isVerbose() const;

    inline void setVerbose(bool verbose);

    inline JobTimer* getJobTimer() const;

    inline void setJobTimer(JobTimer* jobTimer);

    /**
     * Determines whether or not the dependent variables will be set to zero
     * before executing the operation graph
     *
     * @return true if the dependents will be zeroed
     */
    inline bool isZeroDependents() const;

    /**
     * Defines whether or not the dependent variables should be set to zero
     * before executing the operation graph
     *
     * @param true if the dependents should be zeroed
     */
    inline void setZeroDependents(bool zeroDependents);

    inline size_t getOperationTreeVisitId() const;

    inline void startNewOperationTreeVisit();

    inline bool isVisited(const Node& node) const;

    inline void markVisited(const Node& node);

    /**
     * Provides the name used by an atomic function with a given ID.
     *
     * @param id the atomic function ID.
     * @return the atomic function name if it was registered 
     *         or an empty string otherwise.
     */
    inline std::string getAtomicFunctionName(size_t id) const;

    /**
     * Provides a map with all the currently registered atomic functions.
     *
     * @return a map with the atomic function ID as key and the atomic
     *         function as value
     */
    inline const std::map<size_t, CGAbstractAtomicFun<Base>* >& getAtomicFunctions() const;

    /**
     * Provides the maximum forward mode order used by all atomic functions
     * in the last call to ::generateCode
     * (-1 means forward mode not used).
     */
    const std::vector<int>& getExternalFuncMaxForwardOrder() const;

    /**
     * Provides the maximum reverse mode order used by all atomic functions
     * in the last call to ::generateCode
     * (-1 means forward mode not used).
     */
    const std::vector<int>& getExternalFuncMaxReverseOrder() const;

    /**
     * Provides the name used by a loop atomic function with a given ID.
     *
     * @param id the atomic function ID.
     * @return a pointer to the atomic loop function name if it was
     *         registered or nullptr otherwise
     */
    inline const std::string* getLoopName(size_t id) const;

    inline const std::vector<ScopePath>& getScopes() const;

    /**************************************************************************
     *                       Graph management functions
     *************************************************************************/
    /**
     * Finds occurrences of a source code fragment in an operation graph.
     *
     * @param root the operation graph where to search
     * @param target the source code fragment to find in root
     * @param max the maximum number of occurrences of code to find in root
     * @return the paths from root to code
     */
    inline std::vector<SourceCodePath> findPaths(Node& root,
                                                 Node& target,
                                                 size_t max);

    inline BidirGraph<Base> findPathGraph(Node& root,
                                          Node& target) ;

    inline BidirGraph<Base> findPathGraph(Node& root,
                                          Node& target,
                                          size_t& bifurcations,
                                          size_t maxBifurcations = (std::numeric_limits<size_t>::max)());

    /**************************************************************************
     *                       Source code generation
     *************************************************************************/

    /**
     * Creates the source code from the operations registered so far.
     *
     * @param out The output stream where the source code is to be printed.
     * @param lang The targeted language.
     * @param dependent The dependent variables for which the source code
     *                  should be generated. By defining this vector the
     *                  number of operations in the source code can be
     *                  reduced and thus providing a more optimized code.
     * @param nameGen Provides the rules for variable name creation.
     */
    virtual void generateCode(std::ostream& out,
                              Language<Base>& lang,
                              CppAD::vector<CGB>& dependent,
                              VariableNameGenerator<Base>& nameGen,
                              const std::string& jobName = "source");

    virtual void generateCode(std::ostream& out,
                              Language<Base>& lang,
                              std::vector<CGB>& dependent,
                              VariableNameGenerator<Base>& nameGen,
                              const std::string& jobName = "source");

    virtual void generateCode(std::ostream& out,
                              Language<Base>& lang,
                              ArrayView<CGB>& dependent,
                              VariableNameGenerator<Base>& nameGen,
                              const std::string& jobName = "source");

    /**
     * Creates the source code from the operations registered so far.
     *
     * @param out The output stream where the source code is to be printed.
     * @param lang The targeted language.
     * @param dependent The dependent variables for which the source code
     *                  should be generated. By defining this vector the
     *                  number of operations in the source code can be
     *                  reduced and thus providing a more optimized code.
     * @param nameGen Provides the rules for variable name creation.
     * @param atomicFunctions The order of the atomic functions.
     */
    virtual void generateCode(std::ostream& out,
                              Language<Base>& lang,
                              CppAD::vector<CGB>& dependent,
                              VariableNameGenerator<Base>& nameGen,
                              std::vector<std::string>& atomicFunctions,
                              const std::string& jobName = "source");

    virtual void generateCode(std::ostream& out,
                              Language<Base>& lang,
                              std::vector<CGB>& dependent,
                              VariableNameGenerator<Base>& nameGen,
                              std::vector<std::string>& atomicFunctions,
                              const std::string& jobName = "source");

    virtual void generateCode(std::ostream& out,
                              Language<Base>& lang,
                              ArrayView<CGB>& dependent,
                              VariableNameGenerator<Base>& nameGen,
                              std::vector<std::string>& atomicFunctions,
                              const std::string& jobName = "source");

    size_t getTemporaryVariableCount() const;

    size_t getTemporaryArraySize() const;

    size_t getTemporarySparseArraySize() const;

    /**************************************************************************
     *                       Reusing handler and nodes
     *************************************************************************/

    /**
     * Resets this handler for a usage with completely different nodes.
     * @warning all managed memory will be deleted
     */
    virtual void reset();

    /**
     * Resets the previously used dependents and their children so that they
     * can be reused again by this handler.
     */
    inline void resetNodes();

    /**************************************************************************
     *                         access to managed memory
     *************************************************************************/

    /**
     * Creates a shallow clone of an operation node
     */
    inline Node* cloneNode(const Node& n);

    inline Node* makeNode(CGOpCode op);

    inline Node* makeNode(CGOpCode op,
                          const Arg& arg);

    inline Node* makeNode(CGOpCode op,
                          std::vector<Arg>&& args);

    inline Node* makeNode(CGOpCode op,
                          std::vector<size_t>&& info,
                          std::vector<Arg>&& args);

    inline Node* makeNode(CGOpCode op,
                          const std::vector<size_t>& info,
                          const std::vector<Arg>& args);

    inline LoopStartOperationNode<Base>* makeLoopStartNode(Node& indexDcl,
                                                           size_t iterationCount);

    inline LoopStartOperationNode<Base>* makeLoopStartNode(Node& indexDcl,
                                                           IndexOperationNode<Base>& iterCount);

    inline LoopEndOperationNode<Base>* makeLoopEndNode(LoopStartOperationNode<Base>& loopStart,
                                                       const std::vector<Arg >& endArgs);

    inline PrintOperationNode<Base>* makePrintNode(const std::string& before,
                                                   const Arg& arg,
                                                   const std::string& after);

    inline IndexOperationNode<Base>* makeIndexNode(Node& indexDcl);

    inline IndexOperationNode<Base>* makeIndexNode(LoopStartOperationNode<Base>& loopStart);

    inline IndexOperationNode<Base>* makeIndexNode(IndexAssignOperationNode<Base>& indexAssign);

    inline IndexAssignOperationNode<Base>* makeIndexAssignNode(Node& index,
                                                               IndexPattern& indexPattern,
                                                               IndexOperationNode<Base>& index1);

    inline IndexAssignOperationNode<Base>* makeIndexAssignNode(Node& index,
                                                               IndexPattern& indexPattern,
                                                               IndexOperationNode<Base>* index1,
                                                               IndexOperationNode<Base>* index2);

    inline Node* makeIndexDclrNode(const std::string& name);

    /**
     * Provides the current number of OperationNodes created by the model.
     * This number is not the total number of operations in the final
     * model since it also contains Operations nodes marking
     * independent variables and there could be unused operations by
     * the model (dead-code).
     * @return The number of OperationNodes created by the model.
     */
    inline size_t getManagedNodesCount() const;

    /**
     * Provides the OperationNodes created by the model.
     */
    inline const std::vector<Node *>& getManagedNodes() const;

    /**
     * Allows to delete OperationNodes that are managed internally.
     * @warning: This is a dangerous method, make sure these nodes are not used
     *           anywhere else!
     * @param start The index of the first OperationNode to be deleted
     * @param end The index after the last OperationNode to be deleted
     */
    inline void deleteManagedNodes(size_t start,
                                   size_t end);

    /**************************************************************************
     *                           Value generation
     *************************************************************************/
    CGB createCG(const Arg& arg);

    /**************************************************************************
     *                           Loop management
     *************************************************************************/

    const std::map<size_t, LoopModel<Base>*>& getLoops() const;

    inline LoopModel<Base>* getLoop(size_t loopId) const;

    inline size_t addLoopDependentIndexPattern(IndexPattern& jacPattern);

    inline void manageLoopDependentIndexPattern(const IndexPattern* pattern);

    inline size_t addLoopIndependentIndexPattern(IndexPattern& pattern, size_t hint);

    /***********************************************************************
     *                           Index patterns
     **********************************************************************/
    static inline void findRandomIndexPatterns(IndexPattern* ip,
                                               std::set<RandomIndexPattern*>& found);

    /**************************************************************************
     *                      Operation graph manipulation
     *************************************************************************/

    /**
     * Solves an expression (e.g. f(x, y) == 0) for a given variable (e.g. x).
     *
     * @param expression  The original expression (f(x, y))
     * @param var  The variable to solve for
     * @return  The expression for the variable
     * @throws CGException if it is not possible to solve the expression
     */
    inline CGB solveFor(Node& expression,
                             Node& var);

    inline bool isSolvable(Node& expression,
                           Node& var);

    /**
     * Eliminates an independent variable by substitution using the provided
     * dependent variable which is assumed to be a residual of an equation.
     * If successful the model will contain one less independent variable.
     *
     * @param indep The independent variable to eliminate.
     * @param dep The dependent variable representing a residual
     * @param removeFromIndeps Whether or not to immediately remove the
     *                         independent variable from the list of
     *                         independents in the model. The substitution
     *                         operation can only be reversed if the
     *                         variable is not removed.
     * @throws CGException if the dependent variable does not belong to this handler
     */
    inline void substituteIndependent(const CGB& indep,
                                      const CGB& dep,
                                      bool removeFromIndeps = true);

    inline void substituteIndependent(Node& indep,
                                      Node& dep,
                                      bool removeFromIndeps = true);

    /**
     * Reverts a substitution of an independent variable that has not been
     * removed from the list of independents yet.
     * Warning: it does not recover any custom name assigned to the variable.
     *
     * @param indep The independent variable
     * @throws CGException if the dependent variable does not belong to this handler
     */
    inline void undoSubstituteIndependent(Node& indep);

    /**
     * Finalizes the substitution of an independent variable by eliminating
     * it from the list of independents. After this operation the variable
     * substitution cannot be undone.
     *
     * @param indep The independent variable
     * @throws CGException if the dependent variable is not an not an alias or it does not belong to this handler
     */
    inline void removeIndependent(Node& indep);

    /**
     * Adds an operation node to the list of nodes to be deleted when this
     * handler is destroyed.
     *
     * @param code The operation node to be managed.
     * @return true if the node was successfully added to the list or
     *         false if it had already been previously added.
     */
    inline bool manageOperationNodeMemory(Node* code);

protected:

    virtual Node* manageOperationNode(Node* code);

    inline void addVector(CodeHandlerVectorSync<Base>* v);

    inline void removeVector(CodeHandlerVectorSync<Base>* v);

    virtual void markCodeBlockUsed(Node& code);

    inline bool handleTemporaryVarInDiffScopes(Node& code,
                                               size_t oldScope, size_t newScope);

    inline void replaceWithConditionalTempVar(Node& tmp,
                                              IndexOperationNode<Base>& iterationIndexOp,
                                              const std::vector<size_t>& iterationRegions,
                                              ScopeIDType oldScope,
                                              ScopeIDType commonScopeColor);

    inline void updateTemporaryVarInDiffScopes(Node& code);

    inline void restoreTemporaryVar(Node& tmp);

    inline void restoreTemporaryVar(Node* tmp,
                                    Node* opClone);

    inline void updateVarScopeUsage(Node* node,
                                    ScopeIDType usageScope,
                                    ScopeIDType oldUsageScope);

    inline void addScopeToVarOrder(size_t scope,
                                   size_t& e);

    /**
     * Determines the depth of the first different scope from scope paths of
     * two scopes
     *
     * @param color1 scope color 1
     * @param color2 scope color 2
     * @return the depth of the first different scope
     */
    inline size_t findFirstDifferentScope(size_t color1,
                                          size_t color2);

    /**
     * Attempt to reduce the number of ifs when there are consecutive ifs with
     * the same condition
     */
    inline void optimizeIfs();

    inline void replaceScope(Node* node,
                             ScopeIDType oldScope,
                             ScopeIDType newScope);

    /**
     * Removes cyclic dependencies when 'ifs' are merged together.
     * Relative variable order must have already been defined.
     *
     * @param node the node being visited
     * @param scope the scope where the cyclic dependency could appear (or scopes inside it)
     * @param endIf the dependency to remove
     */
    inline void breakCyclicDependency(Node* node,
                                      size_t scope,
                                      Node* endIf);

    inline bool containedInScope(const Node& node,
                                 ScopeIDType scope);

    inline static bool containsArgument(const Node& node,
                                        const Node& arg);

    virtual void registerAtomicFunction(CGAbstractAtomicFun<Base>& atomic);

    /***********************************************************************
     *
     **********************************************************************/
    virtual void checkVariableCreation(Node& code);

    inline void addToEvaluationQueue(Node& arg);

    inline void reduceTemporaryVariables(ArrayView<CGB>& dependent);

    /**
     * Change operation order so that the total number of temporary variables is
     * reduced.
     * @param dependent The vector of dependent variable values
     */
    inline void reorderOperations(ArrayView<CGB>& dependent);

    inline void reorderOperation(Node& node);

    /**
     * Determine the highest location in the evaluation queue of temporary
     * variables used by an operation node in the same scope.
     * @return the highest location of the temporary variables or
     *         the location of node itself if it doesn't use any temporary
     *         variable (in the same scope)
     */
    inline size_t findLastTemporaryLocation(Node& node);

    inline void repositionEvaluationQueue(size_t fromPos,
                                          size_t toPos);

    /**
     * Determines when each temporary variable is last used in the
     * evaluation order
     *
     * @param node The current node for which the number of usages is to be to determined
     */
    inline void determineLastTempVarUsage(Node& node);

    /**
     * Determines relations between variables with an ID
     */
    inline void findVariableDependencies();

    inline void findVariableDependencies(size_t i,
                                         Node& node);

    /**
     * Defines the evaluation order for the code fragments that do not
     * create variables (right hand side variables)
     * @param code The operation just added to the evaluation order
     */
    inline void dependentAdded2EvaluationQueue(Node& node);

    inline void updateEvaluationQueueOrder(Node& node,
                                           size_t newEvalOrder);

    inline bool isIndependent(const Node& arg) const;

    inline bool isTemporary(const Node& arg) const;

    inline static bool isTemporaryArray(const Node& arg);

    inline static bool isTemporarySparseArray(const Node& arg);

    inline static Node* getOperationFromAlias(Node& alias);

    inline size_t getEvaluationOrder(const Node& node) const;

    inline void setEvaluationOrder(Node& node,
                                   size_t order);

    inline size_t getLastUsageEvaluationOrder(const Node& node) const;

    inline void setLastUsageEvaluationOrder(const Node& node,
                                            size_t last);

    /**
     * Provides the total number of times the result of an operation node is
     * being used as an argument for another operation.
     * @return the total usage count
     */
    inline size_t getTotalUsageCount(const Node& node) const;

    inline void setTotalUsageCount(const Node& node,
                                   size_t cout);

    inline void increaseTotalUsageCount(const Node& node);

    inline void resetManagedNodes();

    /**************************************************************************
     *                       Graph management functions
     *************************************************************************/

    inline void findPaths(SourceCodePath& path2node,
                          Node& code,
                          std::vector<SourceCodePath>& found,
                          size_t max);

    static inline std::vector<SourceCodePath> findPathsFromNode(const std::vector<SourceCodePath> nodePaths,
                                                                Node& node);

    /**************************************************************************
     *                        Operation graph manipulation
     *************************************************************************/
    /**
     * Solves an expression (e.g. f(x, y) == 0) for a given variable (e.g. x)
     * The variable can appear only once in the expression.
     * This is also known as isolation.
     *
     * @param path  The path from the equation residual to the variable
     * @return  The expression for the variable
     * @throws CGException if it is not possible to solve the expression
     */
    inline CGB solveFor(const SourceCodePath& path);

    /**
     * Reduces the number of occurrences of a variable in an equation.
     * For instance:
     *  f(x,y) = x + y + x
     * could become
     *  f(x,y) = 2 * x + y
     *
     * @param expression  The original expression (f(x, y))
     * @param path1  A path from the equation residual to where the variable
     *               is used.
     * @param path2  A different path from the equation residual to where the
     *               variable is used.
     * @return  The new expression for the equation
     * @throws CGException if it is not possible to combine the multiple
     *                     occurrences of the variable
     */
    inline CGB collectVariable(Node& expression,
                               const SourceCodePath& path1,
                               const SourceCodePath& path2,
                               size_t bifPos);

    inline CGB collectVariableAddSub(const SourceCodePath& pathLeft,
                                     const SourceCodePath& pathRight);

    inline bool isCollectableVariableAddSub(const SourceCodePath& pathLeft,
                                            const SourceCodePath& pathRight,
                                            bool throwEx);

    inline bool isSolvable(const SourceCodePath& path) const;

    /**************************************************************************
     *                     Loop related structure/methods
     *************************************************************************/
    struct LoopData {
        // maps the loop ids of the loop atomic functions
        std::map<size_t, LoopModel<Base>*> loopModels;
        std::vector<LoopEndOperationNode<Base>*> endNodes;
        // the used indexes
        std::set<const Node*> indexes;
        // the used random index patterns
        std::set<RandomIndexPattern*> indexRandomPatterns;
        //
        std::vector<IndexPattern*> dependentIndexPatterns;
        std::vector<const IndexPattern*> dependentIndexPatternManaged; // garbage collection
        std::vector<IndexPattern*> independentIndexPatterns;
        // variables used inside a loop which are assigned outside (for different loop depths)
        std::vector<std::set<Node*> > outerVars;
        // the current loop depth (-1 means no loop)
        int depth;
        // the evaluation order of the loop start for each loop depth
        std::vector<size_t> startEvalOrder;

        inline LoopData() :
            depth(-1) {
        }

        inline void prepare4NewSourceGen();

        inline void reset();

        /**
         * Provides the name used by a loop atomic function with a given ID.
         *
         * @param id the atomic function ID.
         * @return a pointer to the atomic loop function name if it was
         *         registered, nullptr otherwise
         */
        inline const std::string* getLoopName(size_t id) const;

        inline void registerModel(LoopModel<Base>& loop);

        inline LoopModel<Base>* getLoop(size_t loopId) const;

        size_t addDependentIndexPattern(IndexPattern& jacPattern);

        void manageDependentIndexPattern(const IndexPattern* pattern);

        size_t addIndependentIndexPattern(IndexPattern& pattern, size_t hint);

        void addLoopEndNode(Node& node);
    };

    /**************************************************************************
     *                                friends
     *************************************************************************/
    friend class CG<Base>;
    friend class CGAbstractAtomicFun<Base>;
    friend class BaseAbstractAtomicFun<Base>;
    friend class LoopModel<Base>;

};

} // END cg namespace
} // END CppAD namespace

#endif
