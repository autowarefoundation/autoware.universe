#ifndef CPPAD_CG_CODE_HANDLER_IMPL_INCLUDED
#define CPPAD_CG_CODE_HANDLER_IMPL_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

template<class Base>
CodeHandler<Base>::CodeHandler(size_t varCount) :
        _idVisit(1),
        _idCount(1),
        _idArrayCount(1),
        _idSparseArrayCount(1),
        _idAtomicCount(1),
        _dependents(nullptr),
        _lastVisit(*this),
        _scope(*this),
        _evaluationOrder(*this),
        _lastUsageOrder(*this),
        _totalUseCount(*this),
        _operationCount(*this),
        _varId(*this),
        _scopedVariableOrder(1),
        _atomicFunctionsOrder(nullptr),
        _used(false),
        _reuseIDs(true),
        _scopeColorCount(0),
        _currentScopeColor(0),
        _lang(nullptr),
        _minTemporaryVarID(0),
        _zeroDependents(false),
        _verbose(false),
        _jobTimer(nullptr) {
    _codeBlocks.reserve(varCount);
    //_variableOrder.reserve(1 + varCount / 3);
    _scopedVariableOrder[0].reserve(1 + varCount / 3);

    _auxIndexI = makeIndexDclrNode("i");
    _auxIterationIndexOp = makeIndexNode(*_auxIndexI);
}

template<class Base>
inline CodeHandler<Base>::~CodeHandler() {
    reset();

    for (auto* v : _managedVectors) {
        v->handler_ = nullptr;
    }
}

template<class Base>
inline void CodeHandler<Base>::setReuseVariableIDs(bool reuse) {
    _reuseIDs = reuse;
}

template<class Base>
inline bool CodeHandler<Base>::isReuseVariableIDs() const {
    return _reuseIDs;
}

template<class Base>
inline void CodeHandler<Base>::makeVariables(std::vector<AD<CGB> >& variables) {
    for (auto& v : variables) {
        makeVariable(v);
    }
}

template<class Base>
inline void CodeHandler<Base>::makeVariable(AD<CGB>& variable) {
    CGB v;
    makeVariable(v); // make it a codegen variable
    variable = v; // variable id now the same as v
}

template<class Base>
inline void CodeHandler<Base>::makeVariable(CGB& variable) {
    _independentVariables.push_back(makeNode(CGOpCode::Inv));
    variable.makeVariable(*_independentVariables.back());
}

template<class Base>
size_t CodeHandler<Base>::getIndependentVariableSize() const {
    return _independentVariables.size();
}

template<class Base>
inline const CodeHandlerVector<Base, size_t>& CodeHandler<Base>::getVariablesIDs() const {
    return _varId;
}

template<class Base>
inline size_t CodeHandler<Base>::getMaximumVariableID() const {
    return _idCount;
}

template<class Base>
inline bool CodeHandler<Base>::isVerbose() const {
    return _verbose;
}

template<class Base>
inline void CodeHandler<Base>::setVerbose(bool verbose) {
    _verbose = verbose;
}

template<class Base>
inline JobTimer* CodeHandler<Base>::getJobTimer() const {
    return _jobTimer;
}

template<class Base>
inline void CodeHandler<Base>::setJobTimer(JobTimer* jobTimer) {
    _jobTimer = jobTimer;
}

template<class Base>
inline bool CodeHandler<Base>::isZeroDependents() const {
    return _zeroDependents;
}

template<class Base>
inline void CodeHandler<Base>::setZeroDependents(bool zeroDependents) {
    _zeroDependents = zeroDependents;
}

template<class Base>
size_t CodeHandler<Base>::getIndependentVariableIndex(const Node& var) const {
    CPPADCG_ASSERT_UNKNOWN(var.getOperationType() == CGOpCode::Inv);

    auto it = std::find(_independentVariables.begin(), _independentVariables.end(), &var);
    if (it == _independentVariables.end()) {
        throw CGException("Variable not found in the independent variable vector");
    }

    return it - _independentVariables.begin();
}

template<class Base>
inline size_t CodeHandler<Base>::getOperationTreeVisitId() const {
    return _idVisit;
}

template<class Base>
inline void CodeHandler<Base>::startNewOperationTreeVisit() {
    assert(_idVisit < (std::numeric_limits<size_t>::max)());

    _lastVisit.adjustSize();
    _idVisit++;
}

template<class Base>
inline bool CodeHandler<Base>::isVisited(const Node& node) const {
    size_t p = node.getHandlerPosition();
    return p < _lastVisit.size() && _lastVisit[node] == _idVisit;
}

template<class Base>
inline void CodeHandler<Base>::markVisited(const Node& node) {
    _lastVisit.adjustSize(node);
    _lastVisit[node] = _idVisit;
}

template<class Base>
inline std::string CodeHandler<Base>::getAtomicFunctionName(size_t id) const {
    typename std::map<size_t, CGAbstractAtomicFun<Base>*>::const_iterator it;
    it = _atomicFunctions.find(id);
    if (it != _atomicFunctions.end())
        return it->second->atomic_name();
    else
        return std::string();
}

template<class Base>
inline const std::map<size_t, CGAbstractAtomicFun<Base>* >& CodeHandler<Base>::getAtomicFunctions() const {
    return _atomicFunctions;
}

template<class Base>
const std::vector<int>& CodeHandler<Base>::getExternalFuncMaxForwardOrder() const {
    return _atomicFunctionsMaxForward;
}

template<class Base>
const std::vector<int>& CodeHandler<Base>::getExternalFuncMaxReverseOrder() const {
    return _atomicFunctionsMaxReverse;
}

template<class Base>
inline const std::string* CodeHandler<Base>::getLoopName(size_t id) const {
    return _loops.getLoopName(id);
}

template<class Base>
inline const std::vector<typename CodeHandler<Base>::ScopePath>&
CodeHandler<Base>::getScopes() const {
    return _scopes;
}

template<class Base>
void CodeHandler<Base>::generateCode(std::ostream& out,
                                     Language<Base>& lang,
                                     CppAD::vector<CGB>& dependent,
                                     VariableNameGenerator<Base>& nameGen,
                                     const std::string& jobName) {
    ArrayView<CGB> deps(dependent);
    generateCode(out, lang, deps, nameGen, jobName);
}

template<class Base>
void CodeHandler<Base>::generateCode(std::ostream& out,
                                     Language<Base>& lang,
                                     std::vector<CGB>& dependent,
                                     VariableNameGenerator<Base>& nameGen,
                                     const std::string& jobName) {
    ArrayView<CGB> deps(dependent);
    generateCode(out, lang, deps, nameGen, jobName);
}

template<class Base>
void CodeHandler<Base>::generateCode(std::ostream& out,
                                     Language<Base>& lang,
                                     ArrayView<CGB>& dependent,
                                     VariableNameGenerator<Base>& nameGen,
                                     const std::string& jobName) {
    std::vector<std::string> atomicFunctions;
    generateCode(out, lang, dependent, nameGen, atomicFunctions, jobName);
}

template<class Base>
void CodeHandler<Base>::generateCode(std::ostream& out,
                                     Language<Base>& lang,
                                     CppAD::vector<CGB>& dependent,
                                     VariableNameGenerator<Base>& nameGen,
                                     std::vector<std::string>& atomicFunctions,
                                     const std::string& jobName) {
    ArrayView<CGB> deps(dependent);
    generateCode(out, lang, deps, nameGen, atomicFunctions, jobName);
}

template<class Base>
void CodeHandler<Base>::generateCode(std::ostream& out,
                                     Language<Base>& lang,
                                     std::vector<CGB>& dependent,
                                     VariableNameGenerator<Base>& nameGen,
                                     std::vector<std::string>& atomicFunctions,
                                     const std::string& jobName) {
    ArrayView<CGB> deps(dependent);
    generateCode(out, lang, deps, nameGen, atomicFunctions, jobName);
}

template<class Base>
void CodeHandler<Base>::generateCode(std::ostream& out,
                                     Language<Base>& lang,
                                     ArrayView<CGB>& dependent,
                                     VariableNameGenerator<Base>& nameGen,
                                     std::vector<std::string>& atomicFunctions,
                                     const std::string& jobName) {
    using namespace std::chrono;
    steady_clock::time_point beginTime;

    if (_jobTimer != nullptr) {
        _jobTimer->startingJob("source for '" + jobName + "'");
    } else if (_verbose) {
        std::cout << "generating source for '" << jobName << "' ... ";
        std::cout.flush();
        beginTime = steady_clock::now();
    }

    _lang = &lang;
    _idCount = 1;
    _idArrayCount = 1;
    _idSparseArrayCount = 1;
    _idAtomicCount = 1;
    _dependents = &dependent;
    _atomicFunctionsOrder = &atomicFunctions;
    _atomicFunctionsMaxForward.resize(atomicFunctions.size());
    _atomicFunctionsMaxReverse.resize(atomicFunctions.size());
    _atomicFunctionName2Index.clear();
    _loops.prepare4NewSourceGen();
    _scopeColorCount = 0;
    _currentScopeColor = 0;
    _scopes.reserve(4);
    _scopes.resize(1);
    _alteredNodes.clear();
    _evaluationOrder.adjustSize();
    _lastUsageOrder.adjustSize();
    _totalUseCount.adjustSize();
    _operationCount.adjustSize();
    _varId.adjustSize();
    _scope.adjustSize();

    for (size_t i = 0; i < atomicFunctions.size(); i++) {
        _atomicFunctionName2Index[atomicFunctions[i]] = i;
    }
    std::fill(_atomicFunctionsMaxForward.begin(), _atomicFunctionsMaxForward.end(), -1);
    std::fill(_atomicFunctionsMaxReverse.begin(), _atomicFunctionsMaxReverse.end(), -1);

    if (_used) {
        resetManagedNodes();
    }
    _used = true;

    /**
     * the first variable IDs are for the independent variables
     */
    size_t n = _independentVariables.size();
    for (size_t j = 0; j < n; j++) {
        _varId[*_independentVariables[j]] = _idCount++;
    }

    size_t m = dependent.size();
    for (size_t i = 0; i < m; i++) {
        Node* node = dependent[i].getOperationNode();
        if (node != nullptr && _varId[*node] == 0) {
            _varId[*node] = _idCount++;
        }
    }

    _minTemporaryVarID = _idCount;

    /**
     * determine the number of times each variable is used
     */
    for (size_t i = 0; i < m; i++) {
        Node* node = dependent[i].getOperationNode();
        if (node != nullptr) {
            markCodeBlockUsed(*node);
        }
    }

    /**
     * determine the variable creation order
     */
    _scopedVariableOrder.reserve(std::max<size_t>(size_t(1), _scopes.size()) + 10); // some additional scopes might still be added

    startNewOperationTreeVisit();

    for (size_t i = 0; i < m; i++) {
        CGB& var = dependent[i];
        if (var.getOperationNode() != nullptr) {
            Node& code = *var.getOperationNode();
            if (!isVisited(code)) {
                // dependencies not visited yet
                checkVariableCreation(code);

                // make sure new temporary variables are NOT created for
                // the independent variables and that a dependency did
                // not use it first
                if ((_varId[code] == 0 || !isIndependent(code)) && !isVisited(code)) {
                    addToEvaluationQueue(code);
                }
            }
            markVisited(code);
        }
    }

    /**
     * Generate flat variable order (without scopes)
     */
    if (_scopedVariableOrder.size() == 1) {
        _variableOrder.swap(_scopedVariableOrder[0]); // most common situation
    } else {
        optimizeIfs(); // reduce the number of adjoining ifs

        size_t vosize = 0;
        for (size_t s = 0; s < _scopedVariableOrder.size(); s++) {
            vosize += _scopedVariableOrder[s].size();
        }
        _variableOrder.resize(vosize);

        size_t e = 0;
        addScopeToVarOrder(0, e);

        // if e > vosize then some nodes (marking the beginning of scopes)
        // must have been added more than once
        CPPADCG_ASSERT_UNKNOWN(_variableOrder.size() == e)
    }

    for (size_t p = 0; p < _variableOrder.size(); p++) {
        Node& arg = *_variableOrder[p];
        setEvaluationOrder(arg, p + 1);
        dependentAdded2EvaluationQueue(arg);
    }

    /**
     * Reuse temporary variables
     */
    if (_reuseIDs) {
        reduceTemporaryVariables(dependent);
    }

    /**
     *
     */
    _variableDependencies.clear();
    if(lang.requiresVariableDependencies()) {
        findVariableDependencies();
    }

    nameGen.setTemporaryVariableID(_minTemporaryVarID, _idCount - 1, _idArrayCount - 1, _idSparseArrayCount - 1);

    std::map<std::string, size_t> atomicFunctionName2Id;
    for (const auto& pair: _atomicFunctions) {
        atomicFunctionName2Id[pair.second->atomic_name()] = pair.first;
    }

    std::map<size_t, size_t> atomicFunctionId2Index;
    std::map<size_t, std::string> atomicFunctionId2Name;
    for (size_t i = 0; i < _atomicFunctionsOrder->size(); i++) {
        const std::string& atomicName = (*_atomicFunctionsOrder)[i];
        std::map<std::string, size_t>::const_iterator it = atomicFunctionName2Id.find(atomicName);
        if (it != atomicFunctionName2Id.end()) {
            atomicFunctionId2Index[it->second] = i;
            atomicFunctionId2Name[it->second] = atomicName;
        }
    }

    /**
     * Creates the source code for a specific language
     */

    std::unique_ptr<LanguageGenerationData<Base> > _info(new LanguageGenerationData<Base>(_independentVariables, dependent,
                                                                                          _minTemporaryVarID, _varId, _variableOrder, _variableDependencies,
                                                                                          nameGen,
                                                                                          atomicFunctionId2Index, atomicFunctionId2Name,
                                                                                          _atomicFunctionsMaxForward, _atomicFunctionsMaxReverse,
                                                                                          _reuseIDs,
                                                                                          _loops.indexes, _loops.indexRandomPatterns,
                                                                                          _loops.dependentIndexPatterns, _loops.independentIndexPatterns,
                                                                                          _totalUseCount, _scope, *_auxIterationIndexOp,
                                                                                          _zeroDependents));

    lang.generateSourceCode(out, std::move(_info));

    /**
     * clean-up
     */
    _atomicFunctionName2Index.clear();

    // restore altered nodes
    for (const auto& itAlt : _alteredNodes) {
        Node* tmp = itAlt.first;
        Node* opClone = itAlt.second;
        if (tmp->getOperationType() == CGOpCode::Tmp && !tmp->getInfo().empty()) { // some might have already been restored
            tmp->setOperation(opClone->getOperationType(), opClone->getArguments());
            tmp->getInfo() = opClone->getInfo();
        }
    }
    _alteredNodes.clear();

    if (_jobTimer != nullptr) {
        _jobTimer->finishedJob();
    } else if (_verbose) {
        OStreamConfigRestore osr(std::cout);
        duration<float> dt = steady_clock::now() - beginTime;
        std::cout << "done [" << std::fixed << std::setprecision(3) << dt.count() << "]" << std::endl;
    }
}

template<class Base>
size_t CodeHandler<Base>::getTemporaryVariableCount() const {
    if (_idCount == 1)
        return 0; // no code generated
    else
        return _idCount - _minTemporaryVarID;
}

template<class Base>
size_t CodeHandler<Base>::getTemporaryArraySize() const {
    return _idArrayCount - 1;
}

template<class Base>
size_t CodeHandler<Base>::getTemporarySparseArraySize() const {
    return _idSparseArrayCount - 1;
}

template<class Base>
void CodeHandler<Base>::reset() {
    for (Node* n : _codeBlocks) {
        delete n;
    }
    _codeBlocks.clear();
    _independentVariables.clear();
    _idCount = 1;
    _idArrayCount = 1;
    _idSparseArrayCount = 1;
    _idAtomicCount = 1;

    _loops.reset();

    _used = false;
}

template<class Base>
inline void CodeHandler<Base>::resetNodes() {
    _scope.fill(0);
    _evaluationOrder.fill(0);
    _lastUsageOrder.fill(0);
    _totalUseCount.fill(0);
    _operationCount.fill(0);
    _varId.fill(0);
}

/******************************************************************************
*                            access to managed memory
******************************************************************************/

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::cloneNode(const Node& n) {
    return manageOperationNode(new Node(n));
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::makeNode(CGOpCode op) {
    return manageOperationNode(new Node(this, op));
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::makeNode(CGOpCode op,
                                                        const Arg& arg) {
    return manageOperationNode(new Node(this, op, arg));
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::makeNode(CGOpCode op,
                                                        std::vector<Arg>&& args) {
    return manageOperationNode(new Node(this, op, std::move(args)));
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::makeNode(CGOpCode op,
                                                        std::vector<size_t>&& info,
                                                        std::vector<Arg>&& args) {
    return manageOperationNode(new Node(this, op, std::move(info), std::move(args)));
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::makeNode(CGOpCode op,
                                                        const std::vector<size_t>& info,
                                                        const std::vector<Arg>& args) {
    return manageOperationNode(new Node(this, op, info, args));
}

template<class Base>
inline LoopStartOperationNode<Base>* CodeHandler<Base>::makeLoopStartNode(Node& indexDcl,
                                                                          size_t iterationCount) {
    auto* n = new LoopStartOperationNode<Base>(this, indexDcl, iterationCount);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline LoopStartOperationNode<Base>* CodeHandler<Base>::makeLoopStartNode(Node& indexDcl,
                                                                          IndexOperationNode<Base>& iterCount) {
    auto* n = new LoopStartOperationNode<Base>(this, indexDcl, iterCount);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline LoopEndOperationNode<Base>* CodeHandler<Base>::makeLoopEndNode(LoopStartOperationNode<Base>& loopStart,
                                                                      const std::vector<Arg>& endArgs) {
    auto* n = new LoopEndOperationNode<Base>(this, loopStart, endArgs);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline PrintOperationNode<Base>* CodeHandler<Base>::makePrintNode(const std::string& before,
                                                                  const Arg& arg,
                                                                  const std::string& after) {
    auto* n = new PrintOperationNode<Base>(this, before, arg, after);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline IndexOperationNode<Base>* CodeHandler<Base>::makeIndexNode(Node& indexDcl) {
    auto* n = new IndexOperationNode<Base>(this, indexDcl);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline IndexOperationNode<Base>* CodeHandler<Base>::makeIndexNode(LoopStartOperationNode<Base>& loopStart) {
    auto* n = new IndexOperationNode<Base>(this, loopStart);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline IndexOperationNode<Base>* CodeHandler<Base>::makeIndexNode(IndexAssignOperationNode<Base>& indexAssign) {
    auto* n = new IndexOperationNode<Base>(this, indexAssign);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline IndexAssignOperationNode<Base>* CodeHandler<Base>::makeIndexAssignNode(Node& index,
                                                                              IndexPattern& indexPattern,
                                                                              IndexOperationNode<Base>& index1) {
    auto* n = new IndexAssignOperationNode<Base>(this, index, indexPattern, index1);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline IndexAssignOperationNode<Base>* CodeHandler<Base>::makeIndexAssignNode(Node& index,
                                                                              IndexPattern& indexPattern,
                                                                              IndexOperationNode<Base>* index1,
                                                                              IndexOperationNode<Base>* index2) {
    auto* n = new IndexAssignOperationNode<Base>(this, index, indexPattern, index1, index2);
    manageOperationNode(n);
    return n;
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::makeIndexDclrNode(const std::string& name) {
    CPPADCG_ASSERT_KNOWN(!name.empty(), "index name cannot be empty")
    auto* n = manageOperationNode(new Node(this, CGOpCode::IndexDeclaration));
    n->setName(name);
    return n;
}

template<class Base>
inline size_t CodeHandler<Base>::getManagedNodesCount() const {
    return _codeBlocks.size();
}

template<class Base>
inline const std::vector<OperationNode<Base> *>& CodeHandler<Base>::getManagedNodes() const {
    return _codeBlocks;
}

template<class Base>
inline void CodeHandler<Base>::deleteManagedNodes(size_t start, size_t end) {
    if (start >= end)
        return;

    start = std::min<size_t>(start, _codeBlocks.size());
    end = std::min<size_t>(end, _codeBlocks.size());

    for (size_t i = start; i < end; ++i) {
        delete _codeBlocks[i];
    }
    _codeBlocks.erase(_codeBlocks.begin() + start, _codeBlocks.begin() + end);

    // update positions
    for (size_t i = start; i < _codeBlocks.size(); ++i) {
        _codeBlocks[i]->setHandlerPosition(i);
    }

    for (auto* v : _managedVectors) {
        v->nodesErased(start, end);
    }
}

/******************************************************************************
 *                           Value generation
 *****************************************************************************/
template<class Base>
CG<Base> CodeHandler<Base>::createCG(const Arg& arg) {
    return CGB(arg);
}

/******************************************************************************
 *                                 Index patterns
 *****************************************************************************/
template<class Base>
inline void CodeHandler<Base>::findRandomIndexPatterns(IndexPattern* ip,
                                                       std::set<RandomIndexPattern*>& found) {
    if (ip == nullptr)
        return;

    if (ip->getType() == IndexPatternType::Random1D || ip->getType() == IndexPatternType::Random2D) {
        found.insert(static_cast<RandomIndexPattern*> (ip));
    } else {
        std::set<IndexPattern*> indexes;
        ip->getSubIndexes(indexes);
        for (IndexPattern* sip : indexes) {
            if (sip->getType() == IndexPatternType::Random1D || sip->getType() == IndexPatternType::Random2D)
                found.insert(static_cast<RandomIndexPattern*> (sip));
        }
    }
}

/**************************************************************************
 *                      Operation graph manipulation
 *************************************************************************/
template<class Base>
inline bool CodeHandler<Base>::manageOperationNodeMemory(Node* code) {
    size_t pos = code->getHandlerPosition();
    if (pos >= _codeBlocks.size() || code != _codeBlocks[pos]) {
        manageOperationNode(code);
        return false;
    }
    return true;
}

template<class Base>
OperationNode<Base>* CodeHandler<Base>::manageOperationNode(Node* code) {
    //CPPADCG_ASSERT_UNKNOWN(std::find(_codeBlocks.begin(), _codeBlocks.end(), code) == _codeBlocks.end()); // <<< too great of an impact in performance
    if (_codeBlocks.capacity() == _codeBlocks.size()) {
        _codeBlocks.reserve((_codeBlocks.size() * 3) / 2 + 1);
    }

    code->setHandlerPosition(_codeBlocks.size());
    _codeBlocks.push_back(code);
    return code;
}

template<class Base>
inline void CodeHandler<Base>::addVector(CodeHandlerVectorSync<Base>* v) {
    _managedVectors.insert(v);
}

template<class Base>
inline void CodeHandler<Base>::removeVector(CodeHandlerVectorSync<Base>* v) {
    _managedVectors.erase(v);
}

template<class Base>
void CodeHandler<Base>::markCodeBlockUsed(Node& root) {

    auto startAnalysis = [this](OperationStackData<Base>& stackEl,
                                OperationStack<Base>& stack) {

        auto& code = stackEl.node();

        increaseTotalUsageCount(code);

        CGOpCode op = code.getOperationType();
        if (isIndependent(code)) {
            return false; // nothing to do (go up)

        } else if (op == CGOpCode::Alias) {
            /**
             * Alias operations are always followed so that there is a
             * correct usage count at the operation that it points to
             */
            CPPADCG_ASSERT_UNKNOWN(code.getArguments().size() == 1)
            stack.emplace_back(code, 0, _currentScopeColor);

            return false;

        } else if (getTotalUsageCount(code) == 1) {
            // first time this operation is visited

            size_t previousScope = _currentScopeColor;

            _scope[code] = _currentScopeColor;

            // check if there is a scope change
            if (op == CGOpCode::LoopStart || op == CGOpCode::StartIf || op == CGOpCode::ElseIf ||
                op == CGOpCode::Else) {
                // leaving a scope
                ScopePath& sPath = _scopes[_currentScopeColor];
                CPPADCG_ASSERT_UNKNOWN(sPath.back().beginning == nullptr)
                if (op == CGOpCode::LoopStart || op == CGOpCode::StartIf) {
                    sPath.back().beginning = &code; // save the initial node
                } else {
                    CPPADCG_ASSERT_UNKNOWN(!code.getArguments().empty() &&
                                           code.getArguments()[0].getOperation() != nullptr &&
                                           code.getArguments()[0].getOperation()->getOperationType() ==
                                           CGOpCode::StartIf)
                    sPath.back().beginning = code.getArguments()[0].getOperation(); // save the initial node
                }
                _currentScopeColor = sPath.size() > 1 ? sPath[sPath.size() - 2].color : 0;
            }

            if (op == CGOpCode::LoopEnd || op == CGOpCode::EndIf || op == CGOpCode::ElseIf || op == CGOpCode::Else) {
                // entering a new scope
                _currentScopeColor = ++_scopeColorCount;

                _scopes.resize(_currentScopeColor + 1);
                _scopes[_currentScopeColor] = _scopes[previousScope];

                // change current scope
                if (op == CGOpCode::LoopEnd || op == CGOpCode::EndIf) {
                    // one more scope level
                    _scopes[_currentScopeColor].push_back(ScopePathElement<Base>(_currentScopeColor, &code));
                } else {
                    // same level but different scope
                    _scopes[_currentScopeColor].back() = ScopePathElement<Base>(_currentScopeColor, &code);
                }

                if (op == CGOpCode::LoopEnd) {
                    _loops.addLoopEndNode(code);
                }
            }

            /**
             * Iterate over all arguments with operation nodes
             */
            stack.pushNodeArguments(code, _currentScopeColor);

            return true;

        } else {
            // been to this node before

            if (op == CGOpCode::Tmp && !code.getInfo().empty()) {
                /**
                 * this node was previously altered to ensure that the
                 * evaluation of the expression is only performed for the
                 * required iterations
                 */
                if (_scope[code] == _currentScopeColor) {
                    // outside an if (defined for all iterations)
                    restoreTemporaryVar(code);
                } else {
                    updateTemporaryVarInDiffScopes(code);
                }

            } else if (_scope[code] != _currentScopeColor && op != CGOpCode::LoopIndexedIndep) {
                ScopeIDType oldScope = _scope[code];
                /**
                 * node previously used in a different scope
                 * must make sure it is defined before being used in both
                 * scopes
                 */
                size_t depth = findFirstDifferentScope(oldScope, _currentScopeColor);

                // update the scope where it should be defined
                ScopeIDType newScope;
                if (depth == 0)
                    newScope = 0;
                else
                    newScope = _scopes[_currentScopeColor][depth - 1].color;

                if (oldScope != newScope) {
                    /**
                     * does this variable require a condition based on indexes?
                     */
                    bool addedIf = handleTemporaryVarInDiffScopes(code, oldScope, newScope);

                    if (!addedIf) {
                        _scope[code] = newScope;

                        /**
                         * Must also update the scope of the arguments used by this operation
                         */
                        const std::vector<Arg>& args = code.getArguments();
                        size_t aSize = args.size();
                        for (size_t a = 0; a < aSize; a++) {
                            updateVarScopeUsage(args[a].getOperation(), newScope, oldScope);
                        }
                    }
                }
            }

            return false;
        }
    };


    auto endAnalysis = [this](OperationStackData<Base>& stackEl) {
        // executed after all children have been visited

        Node& code = stackEl.node();
        size_t previousScope = stackEl.parentNodeScope;

        CGOpCode op = code.getOperationType();

        if (op == CGOpCode::Index) {
            const auto& inode = static_cast<const IndexOperationNode <Base>&> (code);
            // indexes that don't depend on a loop start or an index assignment are declared elsewhere
            if (inode.isDefinedLocally()) {
                _loops.indexes.insert(&inode.getIndex());
            }
        } else if (op == CGOpCode::LoopIndexedIndep || op == CGOpCode::LoopIndexedDep ||
                   op == CGOpCode::IndexAssign) {
            IndexPattern* ip;
            if (op == CGOpCode::LoopIndexedDep) {
                size_t pos = code.getInfo()[0];
                ip = _loops.dependentIndexPatterns[pos];
            } else if (op == CGOpCode::LoopIndexedIndep) {
                size_t pos = code.getInfo()[1];
                ip = _loops.independentIndexPatterns[pos];
            } else {
                ip = &static_cast<IndexAssignOperationNode <Base>&> (code).getIndexPattern();
            }

            findRandomIndexPatterns(ip, _loops.indexRandomPatterns);

        } else if (op == CGOpCode::DependentRefRhs) {
            CPPADCG_ASSERT_UNKNOWN(code.getInfo().size() == 1)
            size_t depIndex = code.getInfo()[0];

            CPPADCG_ASSERT_UNKNOWN(_dependents->size() > depIndex)
            Node * depNode = (*_dependents)[depIndex].getOperationNode();
            CPPADCG_ASSERT_UNKNOWN(depNode != nullptr && depNode->getOperationType() != CGOpCode::Inv)

            _varId[code] = _varId[*depNode];
        }

        /**
         * reset scope
         */
        _currentScopeColor = previousScope;

    };

    depthFirstGraphNavigation(root,
                              _currentScopeColor,
                              startAnalysis,
                              endAnalysis,
                              true);
}

template<class Base>
inline bool CodeHandler<Base>::handleTemporaryVarInDiffScopes(Node& code,
                                                              size_t oldScope,
                                                              size_t newScope) {
    if (_currentScopeColor == 0)
        return false;

    /**
     * @TODO allow Array elements to use a CGTmp instead of a CGArrayCreationOp
     */
    CPPADCG_ASSERT_KNOWN(code.getOperationType() != CGOpCode::ArrayCreation, "Not supported yet")
    CPPADCG_ASSERT_KNOWN(code.getOperationType() != CGOpCode::SparseArrayCreation, "Not supported yet")

    /**
     * does this variable require a condition based on indexes?
     */
    std::vector<size_t> iterationRegions;
    Node* bScopeNewEnd = _scopes[_currentScopeColor].back().end;
    Node* bScopeOldEnd = _scopes[oldScope].back().end;

    CGOpCode bNewOp = bScopeNewEnd->getOperationType();
    CGOpCode bOldOp = bScopeOldEnd->getOperationType();

    if ((bNewOp == CGOpCode::EndIf || bNewOp == CGOpCode::Else || bNewOp == CGOpCode::ElseIf) &&
        (bOldOp == CGOpCode::EndIf || bOldOp == CGOpCode::Else || bOldOp == CGOpCode::ElseIf)) {
        // used in 2 different if/else branches

        /**
         * determine the iterations which use this temporary variable
         */
        Node* bScopeNew = bScopeNewEnd->getArguments()[0].getOperation();
        Node* bScopeOld = bScopeOldEnd->getArguments()[0].getOperation();

        IndexOperationNode<Base>* newIterIndexOp = nullptr;
        iterationRegions = ifBranchIterationRanges(bScopeNew, newIterIndexOp);
        CPPADCG_ASSERT_UNKNOWN(iterationRegions.size() >= 2)

        IndexOperationNode<Base>* oldIterIndexOp = nullptr;
        std::vector<size_t> oldIterRegions = ifBranchIterationRanges(bScopeOld, oldIterIndexOp);
        combineOverlapingIterationRanges(iterationRegions, oldIterRegions);
        CPPADCG_ASSERT_UNKNOWN(iterationRegions.size() >= 2)
        CPPADCG_ASSERT_UNKNOWN(newIterIndexOp != nullptr && newIterIndexOp == oldIterIndexOp)

        if (iterationRegions.size() > 2 ||
            iterationRegions[0] != 0 ||
            iterationRegions[1] != (std::numeric_limits<size_t>::max)()) {
            // this temporary variable is not used by all iterations

            replaceWithConditionalTempVar(code, *newIterIndexOp, iterationRegions, oldScope, newScope);
            return true;
        }
    }

    return false;
}

template<class Base>
inline void CodeHandler<Base>::replaceWithConditionalTempVar(Node& tmp,
                                                             IndexOperationNode<Base>& iterationIndexOp,
                                                             const std::vector<size_t>& iterationRegions,
                                                             ScopeIDType oldScope,
                                                             ScopeIDType commonScopeColor) {
    /**
     * clone
     */
    Node* opClone = cloneNode(tmp);

    /**
     * Create condition
     */
    Node* tmpDclVar = makeNode(CGOpCode::TmpDcl);
    Arg tmpArg(*tmpDclVar);

    Node* cond = makeNode(CGOpCode::IndexCondExpr, iterationRegions, {iterationIndexOp});

    // if
    Node* ifStart = makeNode(CGOpCode::StartIf, *cond);

    Node* tmpAssign = makeNode(CGOpCode::LoopIndexedTmp, {tmpArg, *opClone});
    Node* ifAssign = makeNode(CGOpCode::CondResult, {*ifStart, *tmpAssign});

    // end if
    Node* endIf = makeNode(CGOpCode::EndIf, {*ifStart, *ifAssign});

    /**
     * Change original variable
     */
    tmp.setOperation(CGOpCode::Tmp, {tmpArg, *endIf});
    tmp.getInfo().resize(1); // used to mark that this node was altered here

    // created new nodes, must adjust vector sizes
    _scope.adjustSize();
    _lastVisit.adjustSize();
    _scope.adjustSize();
    _evaluationOrder.adjustSize();
    _lastUsageOrder.adjustSize();
    _totalUseCount.adjustSize();
    _operationCount.adjustSize();
    _varId.adjustSize();

    /**
     * add the new scope
     */
    size_t newScopeColor = ++_scopeColorCount;
    _scopes.resize(newScopeColor + 1);
    _scopes[newScopeColor] = _scopes[commonScopeColor];

    // one more scope level
    _scopes[newScopeColor].push_back(ScopePathElement<Base>(newScopeColor, endIf, ifStart));

    // apply scope colors
    _scope[*tmpDclVar] = commonScopeColor;
    _scope[*ifStart] = newScopeColor;
    _scope[*cond] = newScopeColor;
    _scope[*opClone] = newScopeColor;
    _scope[*ifAssign] = newScopeColor;
    _scope[*tmpAssign] = newScopeColor;
    _scope[*endIf] = commonScopeColor;
    _scope[tmp] = commonScopeColor;

    // total usage count
    setTotalUsageCount(*tmpDclVar, 1);
    setTotalUsageCount(*ifStart, 1);
    setTotalUsageCount(*cond, 1);
    setTotalUsageCount(*opClone, 1);
    setTotalUsageCount(*ifAssign, 1);
    setTotalUsageCount(*tmpAssign, 1);
    setTotalUsageCount(*endIf, 1);

    /**
     * Must also update the scope of the arguments used by this operation
     */
    const std::vector<Arg>& cargs = opClone->getArguments();
    size_t aSize = cargs.size();
    for (size_t a = 0; a < aSize; a++) {
        updateVarScopeUsage(cargs[a].getOperation(), newScopeColor, oldScope);
    }

    _alteredNodes.push_back(std::make_pair(&tmp, opClone));
}

template<class Base>
inline void CodeHandler<Base>::updateTemporaryVarInDiffScopes(Node& code) {
    if (_scope[code] != _currentScopeColor) {
        return; //nothing to change
    }

    /**
     * does this variable require a condition based on indexes?
     */
    if (_currentScopeColor == 0)
        restoreTemporaryVar(code);

    /**
     * Determine if it should be moved into a different scope
     * so that it is defined before being used in both
     * scopes
     */
    ScopeIDType oldScope = _scope[code];

    size_t depth = findFirstDifferentScope(oldScope, _currentScopeColor);

    // update the scope where it should be defined
    ScopeIDType newScope = depth == 0 ? 0 : _scopes[_currentScopeColor][depth - 1].color;

    /**
     * does this variable require a condition based on indexes?
     */
    std::vector<size_t> iterationRegions;
    Node* bScopeNewEnd = _scopes[_currentScopeColor].back().end;
    Node* endif = code.getArguments()[0].getOperation();
    CPPADCG_ASSERT_UNKNOWN(endif->getOperationType() == CGOpCode::EndIf)
    Node* bScopeOldEnd = _scopes[_scope[*endif]].back().end;

    CGOpCode bNewOp = bScopeNewEnd->getOperationType();

    if (bNewOp == CGOpCode::EndIf || bNewOp == CGOpCode::Else || bNewOp == CGOpCode::ElseIf) {
        // used in 2 different if/else branches

        /**
         * determine the iterations which use this temporary variable
         */
        Node* bScopeNew = bScopeNewEnd->getArguments()[0].getOperation();
        Node* bScopeOld = bScopeOldEnd->getArguments()[0].getOperation();

        IndexOperationNode<Base>* newIterIndexOp = nullptr;
        iterationRegions = ifBranchIterationRanges(bScopeNew, newIterIndexOp);
        CPPADCG_ASSERT_UNKNOWN(iterationRegions.size() >= 2)

        IndexOperationNode<Base>* oldIterIndexOp = nullptr;
        const std::vector<size_t> oldIterRegions = ifBranchIterationRanges(bScopeOld, oldIterIndexOp);
        combineOverlapingIterationRanges(iterationRegions, oldIterRegions);
        CPPADCG_ASSERT_UNKNOWN(iterationRegions.size() >= 2)
        CPPADCG_ASSERT_UNKNOWN(newIterIndexOp != nullptr && newIterIndexOp == oldIterIndexOp)

        if (iterationRegions.size() == 2 &&
            (iterationRegions[0] == 0 ||
             iterationRegions[1] == (std::numeric_limits<size_t>::max)())) {
            // this temporary variable is used by all iterations
            // there is no need for an 'if'
            restoreTemporaryVar(code);

        } else if (oldIterRegions != iterationRegions) {
            Node* cond = bScopeOld->getArguments()[0].getOperation();
            CPPADCG_ASSERT_UNKNOWN(cond->getOperationType() == CGOpCode::IndexCondExpr)
            cond->getInfo() = iterationRegions;
        }

    }

    if (oldScope != newScope) {
        _scope[code] = newScope;
        /**
         * Must also update the scope of the arguments used by this operation
         */
        const std::vector<Arg>& cargs = code.getArguments();
        size_t aSize = cargs.size();
        for (size_t a = 0; a < aSize; a++) {
            updateVarScopeUsage(cargs[a].getOperation(), newScope, oldScope);
        }
    }

}

template<class Base>
inline void CodeHandler<Base>::restoreTemporaryVar(Node& tmp) {
    CPPADCG_ASSERT_UNKNOWN(tmp.getOperationType() == CGOpCode::Tmp && !tmp.getInfo().empty())

    Node* endIf = tmp.getArguments()[1].getOperation();
    Node* ifAssign = endIf->getArguments()[1].getOperation();
    Node* tmpAssign = ifAssign->getArguments()[1].getOperation();
    Node* opClone = tmpAssign->getArguments()[1].getOperation();
    tmp.setOperation(opClone->getOperationType(), opClone->getArguments());
    tmp.getInfo() = opClone->getInfo();

    _scope[tmp] = _currentScopeColor;

    /**
     * Must also update the scope of the arguments used by this operation
     */
    const std::vector<Arg>& args = tmp.getArguments();
    size_t aSize = args.size();
    for (size_t a = 0; a < aSize; a++) {
        updateVarScopeUsage(args[a].getOperation(), _currentScopeColor, _scope[*opClone]);
    }
}

template<class Base>
inline void CodeHandler<Base>::restoreTemporaryVar(Node* tmp,
                                                   Node* opClone) {
    CPPADCG_ASSERT_UNKNOWN(tmp->getOperationType() == CGOpCode::Tmp && !tmp->getInfo().empty())

    tmp->setOperation(opClone->getOperationType(), opClone->getArguments());
    tmp->getInfo() = opClone->getInfo();

    _scope[*tmp] = _currentScopeColor;

    /**
     * Must also update the scope of the arguments used by this operation
     */
    const std::vector<Arg>& args = tmp->getArguments();
    size_t aSize = args.size();
    for (size_t a = 0; a < aSize; a++) {
        updateVarScopeUsage(args[a].getOperation(), _currentScopeColor, _scope[*opClone]);
    }
}

template<class Base>
inline void CodeHandler<Base>::updateVarScopeUsage(Node* node,
                                                   ScopeIDType usageScope,
                                                   ScopeIDType oldUsageScope) {
    if (node == nullptr || _scope[*node] == usageScope)
        return;


    ScopeIDType oldScope = _scope[*node];
    ScopeIDType newScope;

    if (oldScope == oldUsageScope) {
        newScope = usageScope;
    } else {
        size_t depth = findFirstDifferentScope(oldScope, usageScope);

        newScope = (depth == 0) ? 0 : _scopes[usageScope][depth - 1].color;
    }

    if (newScope == oldScope)
        return;

    _scope[*node] = newScope;

    const std::vector<Arg>& args = node->getArguments();
    size_t aSize = args.size();
    for (size_t a = 0; a < aSize; a++) {
        updateVarScopeUsage(args[a].getOperation(), newScope, oldScope);
    }
}

template<class Base>
inline void CodeHandler<Base>::addScopeToVarOrder(size_t scope,
                                                  size_t& e) {
    std::vector<Node*>& vorder = _scopedVariableOrder[scope];

    const size_t vsize = vorder.size();
    for (size_t p = 0; p < vsize; p++) {
        Node* node = vorder[p];
        CGOpCode op = node->getOperationType();

        if (op == CGOpCode::LoopEnd || op == CGOpCode::EndIf || op == CGOpCode::ElseIf || op == CGOpCode::Else) {
            CPPADCG_ASSERT_UNKNOWN(!node->getArguments().empty())

            Node* beginScopeNode = node->getArguments()[0].getOperation();
            CPPADCG_ASSERT_UNKNOWN(beginScopeNode != nullptr)

            addScopeToVarOrder(_scope[*beginScopeNode], e);
        }

        //std::cout << "e:" << e << "  " << vorder[p] << "  scope:" << scope << "  p:" << p << "  " << *vorder[p] << std::endl;
        _variableOrder[e++] = vorder[p];
    }
}

template<class Base>
inline size_t CodeHandler<Base>::findFirstDifferentScope(size_t color1,
                                                         size_t color2) {
    CPPADCG_ASSERT_UNKNOWN(color1 < _scopes.size())
    CPPADCG_ASSERT_UNKNOWN(color2 < _scopes.size())

    ScopePath& scopePath1 = _scopes[color1];
    ScopePath& scopePath2 = _scopes[color2];

    size_t s1 = scopePath1.size();
    size_t s2 = scopePath2.size();
    size_t depth;
    for (depth = 0; depth < s1 && depth < s2; depth++) {
        if (scopePath1[depth].color != scopePath2[depth].color) {
            break;
        }
    }

    return depth;
}

template<class Base>
inline void CodeHandler<Base>::optimizeIfs() {
    if (_scopedVariableOrder.size() < 3)
        return; // there has to be at least 2 ifs

    for (size_t scope = 0; scope < _scopedVariableOrder.size(); scope++) {
        std::vector<Node*>& vorder = _scopedVariableOrder[scope];

        for (long p = vorder.size() - 1; p > 0; p--) {
            Node* endIf = vorder[p];
            if (endIf->getOperationType() != CGOpCode::EndIf)
                continue;

            long p1 = p - 1;
            while (p1 >= 0) {
                if (vorder[p1]->getOperationType() == CGOpCode::TmpDcl) {
                    p1--;
                } else {
                    break;
                }
            }
            Node* endIf1 = vorder[p1];
            if (endIf1->getOperationType() != CGOpCode::EndIf)
                continue;

            // 2 consecutive ifs
            Node* startIf = endIf->getArguments()[0].getOperation();
            Node* startIf1 = endIf1->getArguments()[0].getOperation();
            if (startIf->getOperationType() != CGOpCode::StartIf || startIf1->getOperationType() != CGOpCode::StartIf)
                continue;

            Node* cond = startIf->getArguments()[0].getOperation();
            Node* cond1 = startIf1->getArguments()[0].getOperation();

            CPPADCG_ASSERT_UNKNOWN(cond->getOperationType() == CGOpCode::IndexCondExpr || cond1->getOperationType() == CGOpCode::IndexCondExpr)
            if (cond->getInfo() == cond1->getInfo()) {
                /**
                 * same condition -> combine the contents into a single if
                 */
                const std::vector<Arg>& eArgs = endIf->getArguments();
                std::vector<Arg>& eArgs1 = endIf1->getArguments();

                ScopeIDType ifScope = _scope[*startIf];
                ScopeIDType ifScope1 = _scope[*startIf1];
                std::vector<Node*>& vorderIf = _scopedVariableOrder[ifScope];
                std::vector<Node*>& vorderIf1 = _scopedVariableOrder[ifScope1];

                startNewOperationTreeVisit();

                // break cycles caused by dependencies on the previous if
                for (size_t a = 1; a < eArgs.size(); a++) { // exclude the initial startIf
                    CPPADCG_ASSERT_UNKNOWN(eArgs[a].getOperation() != nullptr && eArgs[a].getOperation()->getOperationType() == CGOpCode::CondResult)
                    breakCyclicDependency(eArgs[a].getOperation(), ifScope, endIf1);
                    replaceScope(eArgs[a].getOperation(), ifScope, ifScope1); // update scope
                }

                vorderIf1.insert(vorderIf1.end(), vorderIf.begin() + 1, vorderIf.end()); // exclude the initial startIf

                vorderIf.clear();

                // update startIf
                for (size_t a = 1; a < eArgs.size(); a++) { // exclude the initial startIf
                    CPPADCG_ASSERT_UNKNOWN(eArgs[a].getOperation() != nullptr && eArgs[a].getOperation()->getOperationType() == CGOpCode::CondResult)
                    eArgs[a].getOperation()->getArguments()[0] = Arg(*startIf1);
                }

                // update endIf
                eArgs1.insert(eArgs1.end(), eArgs.begin() + 1, eArgs.end());

                // replace endIf
                endIf->setOperation(CGOpCode::Alias, {*endIf1});

                // remove one of the ifs
                vorder.erase(vorder.begin() + p);

                // move nodes in scope containing the ifs
                for (long pp = p1; pp < p - 1; pp++) {
                    vorder[pp] = vorder[pp + 1];
                }
                vorder[p - 1] = endIf1;
            }
        }
    }
}

template<class Base>
inline void CodeHandler<Base>::replaceScope(Node* node,
                                            ScopeIDType oldScope,
                                            ScopeIDType newScope) {
    if (node == nullptr || _scope[*node] != oldScope)
        return;

    _scope[*node] = newScope;

    const std::vector<Arg>& args = node->getArguments();
    for (size_t a = 0; a < args.size(); a++) {
        replaceScope(args[a].getOperation(), oldScope, newScope);
    }
}

template<class Base>
inline void CodeHandler<Base>::breakCyclicDependency(Node* node,
                                                     size_t scope,
                                                     Node* endIf) {
    if (node == nullptr || isVisited(*node))
        return;

    markVisited(*node);

    CGOpCode op = node->getOperationType();
    std::vector<Arg>& args = node->getArguments();

    if (op == CGOpCode::Tmp && args.size() > 1) {
        Node* arg = args[1].getOperation();
        if (arg == endIf) {
            // a dependency on LoopIndexedTmp could be added but
            // it is not required since variable order was already decided
            args.erase(args.begin() + 1);
        }
    }

    if (!containedInScope(*node, scope)) {
        return;
    }

    for (size_t a = 0; a < args.size(); a++) {
        Node* arg = args[a].getOperation();
        if (arg == endIf) {
            if (op == CGOpCode::StartIf || op == CGOpCode::LoopStart) {
                args.erase(args.begin() + a);
                a--;
            }
        } else {
            breakCyclicDependency(arg, scope, endIf);
        }
    }
}

template<class Base>
inline bool CodeHandler<Base>::containedInScope(const Node& node,
                                                ScopeIDType scope) {
    ScopeIDType nScope = _scope[node];
    if (nScope == scope)
        return true;

    return _scopes[nScope].size() >= _scopes[scope].size() &&
           _scopes[nScope][_scopes[scope].size() - 1].color == scope;
}

template<class Base>
inline bool CodeHandler<Base>::containsArgument(const Node& node,
                                                const Node& arg) {
    const std::vector<Arg>& args = node.getArguments();
    for (size_t a = 0; a < args.size(); a++) {
        if (args[a].getOperation() == &arg) {
            return true;
        }
    }
    return false;
}

template<class Base>
void CodeHandler<Base>::registerAtomicFunction(CGAbstractAtomicFun<Base>& atomic) {
    size_t id = atomic.getId();
    auto it = _atomicFunctions.lower_bound(id);
    if (it == _atomicFunctions.end() || it->first != id) {
        if (it != _atomicFunctions.end()) ++it;
        _atomicFunctions.insert(it, std::pair<size_t, CGAbstractAtomicFun<Base>*>(atomic.getId(), &atomic));
    } else if(it->second != &atomic) {
        throw CGException("The same atomic function ID (", id, ") is being used for different atomic functions: '",
                          atomic.atomic_name(), "' (", &atomic, ") and '", it->second->atomic_name(), "' (", it->second, ").");
    }
}

template<class Base>
void CodeHandler<Base>::checkVariableCreation(Node& root) {

    auto startAnalysis = [this](OperationStackData<Base>& stackEl,
                                OperationStack<Base>& stack) {
        auto& arg = stackEl.node();

        if (isVisited(arg)) {
            return false;
        } else {
            stack.pushNodeArguments(stackEl.node(), 0);
            return true;
        }
    };

    auto completeAnalysis = [this](OperationStackData<Base>& stackEl) {
        auto& arg = stackEl.node();

        if (isVisited(arg)) {
            return;
        }

        CGOpCode aType = arg.getOperationType();

        if (aType == CGOpCode::LoopEnd || aType == CGOpCode::ElseIf ||
            aType == CGOpCode::Else || aType == CGOpCode::EndIf) {
            if (_varId[arg] == 0) {
                // ID value is not really used but must be non-zero
                _varId[arg] = (std::numeric_limits<size_t>::max)();
            }
        } else if (aType == CGOpCode::AtomicForward || aType == CGOpCode::AtomicReverse) {
            /**
             * Save atomic function related information
             */
            CPPADCG_ASSERT_UNKNOWN(arg.getArguments().size() > 1)
            CPPADCG_ASSERT_UNKNOWN(arg.getInfo().size() > 1)
            size_t id = arg.getInfo()[0];

            size_t pos;
            const std::string& atomicName = _atomicFunctions.at(id)->atomic_name();
            std::map<std::string, size_t>::const_iterator itName2Idx;
            itName2Idx = _atomicFunctionName2Index.find(atomicName);

            if (itName2Idx == _atomicFunctionName2Index.end()) {
                pos = _atomicFunctionsOrder->size();
                _atomicFunctionsOrder->push_back(atomicName);
                _atomicFunctionName2Index[atomicName] = pos;
                _atomicFunctionsMaxForward.push_back(-1);
                _atomicFunctionsMaxReverse.push_back(-1);
            } else {
                pos = itName2Idx->second;
            }

            if (aType == CGOpCode::AtomicForward) {
                int p = arg.getInfo()[2];
                _atomicFunctionsMaxForward[pos] = std::max<int>(_atomicFunctionsMaxForward[pos],
                                                                p);
            } else {
                int p = arg.getInfo()[1];
                _atomicFunctionsMaxReverse[pos] = std::max<int>(_atomicFunctionsMaxReverse[pos],
                                                                p);
            }
        }

        /**
         * make sure new temporary variables are NOT created for
         * the independent variables and that a dependency did
         * not use it first
         */
        if (_varId[arg] == 0 || !isIndependent(arg)) {
            auto& code = stackEl.parent();
            size_t argIndex = stackEl.argumentIndex();

            if (aType == CGOpCode::LoopIndexedIndep) {
                // ID value not really used but must be non-zero
                _varId[arg] = (std::numeric_limits<size_t>::max)();
            } else if (aType == CGOpCode::Alias) {
                return; // should never be added to the evaluation queue
            } else if (aType == CGOpCode::Tmp) {
                _varId[arg] = (std::numeric_limits<size_t>::max)();
            } else if (aType == CGOpCode::LoopStart ||
                       aType == CGOpCode::LoopEnd ||
                       aType == CGOpCode::StartIf ||
                       aType == CGOpCode::ElseIf ||
                       aType == CGOpCode::Else ||
                       aType == CGOpCode::EndIf) {
                /**
                 * Operation that mark a change in variable scope
                 * are always added
                 */
                addToEvaluationQueue(arg);
                if (_varId[arg] == 0) {
                    // ID value is not really used but must be non-zero
                    _varId[arg] = (std::numeric_limits<size_t>::max)();
                }
            } else if (aType == CGOpCode::Pri) {
                addToEvaluationQueue(arg);
                if (_varId[arg] == 0) {
                    // ID value is not really used but must be non-zero
                    _varId[arg] = (std::numeric_limits<size_t>::max)();
                }
            } else if (aType == CGOpCode::TmpDcl) {
                addToEvaluationQueue(arg);

                _varId[arg] = _idCount;
                _idCount++;

            } else {

                // determine the number of operations required to compute this variable
                size_t& opCount = _operationCount[arg];

                opCount = 1;
                for (const auto& a: arg) {
                    if (a.getOperation() != nullptr) {
                        auto& n = *a.getOperation();
                        if (_varId[n] == 0) {
                            opCount += _operationCount[n];
                        }
                    }
                }

                // determine if this variable should be temporary/dependent variable
                if (_lang->createsNewVariable(arg, getTotalUsageCount(arg), opCount) ||
                    _lang->requiresVariableArgument(code.getOperationType(), argIndex)) {

                    addToEvaluationQueue(arg);

                    if (_varId[arg] == 0) {
                        if (aType == CGOpCode::AtomicForward ||
                            aType == CGOpCode::AtomicReverse) {
                            _varId[arg] = _idAtomicCount;
                            _idAtomicCount++;
                        } else if (aType == CGOpCode::LoopIndexedDep ||
                                   aType == CGOpCode::LoopIndexedTmp) {
                            // ID value not really used but must be non-zero
                            _varId[arg] = (std::numeric_limits<size_t>::max)();
                        } else if (aType == CGOpCode::ArrayCreation) {
                            // a temporary array
                            size_t arraySize = arg.getArguments().size();
                            _varId[arg] = _idArrayCount;
                            _idArrayCount += arraySize;
                        } else if (aType == CGOpCode::SparseArrayCreation) {
                            // a temporary array
                            size_t nnz = arg.getArguments().size();
                            _varId[arg] = _idSparseArrayCount;
                            _idSparseArrayCount += nnz;
                        } else {
                            // a single temporary variable
                            _varId[arg] = _idCount;
                            _idCount++;
                        }
                    }
                }
            }

        }

        markVisited(arg);
    };

    depthFirstGraphNavigation(root,
                              _currentScopeColor,
                              startAnalysis,
                              completeAnalysis,
                              false);
}

template<class Base>
inline void CodeHandler<Base>::addToEvaluationQueue(Node& arg) {
    ScopeIDType scope = _scope[arg];
    if (scope >= _scopedVariableOrder.size()) {
        _scopedVariableOrder.resize(scope + 1);
    }

    if (_scopedVariableOrder[scope].empty() &&
        scope != 0 && // the upper most scope does not need any special node at the beginning
        _scopes[scope].back().end->getArguments()[0].getOperation() != &arg) {
        // the first node must be a beginning of a scope
        checkVariableCreation(*_scopes[scope].back().end); // go inside a scope from the end
    }

    // must be after checkVariableCreation() because _scopedVariableOrder might be resized
    std::vector<Node*>& varOrder = _scopedVariableOrder[scope];

    if (varOrder.size() == varOrder.capacity()) {
        varOrder.reserve((varOrder.size() * 3) / 2 + 1);
    }

    varOrder.push_back(&arg);
}

template<class Base>
inline void CodeHandler<Base>::reduceTemporaryVariables(ArrayView<CGB>& dependent) {

    reorderOperations(dependent);

    /**
     * determine the last line where each temporary variable is used
     */
    startNewOperationTreeVisit();

    for (size_t i = 0; i < dependent.size(); i++) {
        Node* node = dependent[i].getOperationNode();
        if (node != nullptr) {
            if (!isVisited(*node)) {
                // dependencies not visited yet
                determineLastTempVarUsage(*node);
            }
            markVisited(*node);
        }
    }

    // where temporary variables can be released
    std::vector<std::vector<Node*>> tempVarRelease(_variableOrder.size());
    for (size_t i = 0; i < _variableOrder.size(); i++) {
        Node* var = _variableOrder[i];
        if (isTemporary(*var) || isTemporaryArray(*var) || isTemporarySparseArray(*var)) {
            size_t releaseLocation = getLastUsageEvaluationOrder(*var) - 1;
            tempVarRelease[releaseLocation].push_back(var);
        }
    }


    /**
     * Redefine temporary variable IDs
     */
    std::vector<size_t> freedVariables; // variable IDs no longer in use
    _idCount = _minTemporaryVarID;
    ArrayIdCompresser<Base> arrayComp(_varId, _idArrayCount);
    ArrayIdCompresser<Base> sparseArrayComp(_varId, _idSparseArrayCount);

    for (size_t i = 0; i < _variableOrder.size(); i++) {
        Node& var = *_variableOrder[i];

        const std::vector<Node*>& released = tempVarRelease[i];
        for (size_t r = 0; r < released.size(); r++) {
            if (isTemporary(*released[r])) {
                freedVariables.push_back(_varId[*released[r]]);
            } else if (isTemporaryArray(*released[r])) {
                arrayComp.addFreeArraySpace(*released[r]);
            } else if (isTemporarySparseArray(*released[r])) {
                sparseArrayComp.addFreeArraySpace(*released[r]);
            }
        }

        if (isTemporary(var)) {
            // a single temporary variable
            if (freedVariables.empty()) {
                _varId[var] = _idCount;
                _idCount++;
            } else {
                size_t id = freedVariables.back();
                freedVariables.pop_back();
                _varId[var] = id;
            }
        } else if (isTemporaryArray(var)) {
            // a temporary array
            size_t arrayStart = arrayComp.reserveArraySpace(var);
            _varId[var] = arrayStart + 1;
        } else if (isTemporarySparseArray(var)) {
            // a temporary array
            size_t arrayStart = sparseArrayComp.reserveArraySpace(var);
            _varId[var] = arrayStart + 1;
        }

    }

    _idArrayCount = arrayComp.getIdCount();
    _idSparseArrayCount = sparseArrayComp.getIdCount();
}

template<class Base>
inline void CodeHandler<Base>::reorderOperations(ArrayView<CGB>& dependent) {
    // determine the location of the last temporary variable used for each dependent
    startNewOperationTreeVisit();

    // normal dependent nodes
    for (size_t i = 0; i < dependent.size(); ++i) {
        Node* node = dependent[i].getOperationNode();
        if (node != nullptr) {
            reorderOperation(*node);
        }
    }

    // dependent nodes defined inside loops
    for (const LoopEndOperationNode<Base>* endNode : _loops.endNodes) {
        const std::vector<Arg>& args = endNode->getArguments();
        for (size_t i = 1; i < args.size(); ++i) {
            CPPADCG_ASSERT_UNKNOWN(args[i].getOperation() != nullptr)
            // TODO: also consider CGOpCode::LoopIndexedDep inside a CGOpCode::endIf
            if (args[i].getOperation()->getOperationType() == CGOpCode::LoopIndexedDep) {
                reorderOperation(*args[i].getOperation());
            }
        }
    }
}

template<class Base>
inline void CodeHandler<Base>::reorderOperation(Node& node) {
    /**
     * determine the location of the last temporary variable
     */
    size_t depPos = getEvaluationOrder(node);
    size_t lastTmpPos = depPos;
    if (!isVisited(node)) {
        // dependencies not visited yet
        lastTmpPos = findLastTemporaryLocation(node);
    }
    markVisited(node);

    /**
     * move dependent if beneficial
     */
    if (lastTmpPos == depPos || lastTmpPos + 1 == depPos) {
        return; // should not change location of the evaluation of this dependent
    }

    // should only move if there are temporaries which could use other temporaries released by the dependent
    bool foundTemporaries = false;
    size_t newPos;
    for (size_t l = lastTmpPos + 1; l < depPos; ++l) {
        const auto* n = _variableOrder[l - 1];
        if (isTemporary(*n) || isTemporaryArray(*n) || isTemporarySparseArray(*n)) {
            foundTemporaries = true;
            newPos = l;
            break;
        } else {
            /**
             * Must be in same scope
             */
            CGOpCode op = n->getOperationType();
            if (op == CGOpCode::StartIf) {
                // must not change scope (find the end of this conditional statement)
                ++l;
                while (l < depPos) {
                    const auto* node2 = _variableOrder[l - 1];
                    if (node2->getOperationType() == CGOpCode::EndIf &&
                        node2->getArguments()[0].getOperation() == n) {
                        break; // found the end (returned to the same scope)
                    }
                    ++l;
                }

            } else if (op == CGOpCode::LoopStart) {
                // must not change scope (find the end of this loop statement)
                ++l;
                while (l < depPos) {
                    const auto* node2 = _variableOrder[l - 1];
                    if (node2->getOperationType() == CGOpCode::LoopEnd &&
                        &static_cast<const LoopEndOperationNode<Base>*> (node2)->getLoopStart() == n) {
                        break; // found the end (returned to the same scope)
                    }
                    ++l;
                }
            }
        }
    }

    if (foundTemporaries) {
        // move variables
        repositionEvaluationQueue(depPos, newPos);
    }
}

template<class Base>
inline size_t CodeHandler<Base>::findLastTemporaryLocation(Node& root) {

    size_t depOrder = getEvaluationOrder(root);
    size_t maxTmpOrder = depOrder; // lowest possible value is 1

    auto nodeAnalysis = [&](SimpleOperationStackData<Base>& stackEl,
                            SimpleOperationStack<Base>& stack) {
        auto& node = stackEl.node();

        const auto& args = node.getArguments();

        for (size_t i = 0; i < args.size(); ++i) {
            if (args[i].getOperation() == nullptr) {
                continue;
            }

            Node& arg = *args[i].getOperation();
            CGOpCode aOp = arg.getOperationType();

            if (aOp == CGOpCode::LoopEnd || aOp == CGOpCode::EndIf || aOp == CGOpCode::ElseIf || aOp == CGOpCode::Else) {
                continue; //should not move variables to a different scope
            }

            if (aOp == CGOpCode::Index) {
                size_t iorder = getEvaluationOrder(static_cast<IndexOperationNode <Base>&> (arg).getIndexCreationNode());
                if (iorder > maxTmpOrder)
                    maxTmpOrder = iorder;

            } else if (getEvaluationOrder(arg) == depOrder) {
                // dependencies not visited yet
                stack.emplace_back(node, i);

            } else {
                // no need to visit dependencies
                if (getEvaluationOrder(arg) > maxTmpOrder)
                    maxTmpOrder = getEvaluationOrder(arg);
            }

        }
    };

    depthFirstGraphNavigation(root, nodeAnalysis, true);

    return maxTmpOrder;
}

template<class Base>
inline void CodeHandler<Base>::repositionEvaluationQueue(size_t fromPos, size_t toPos) {
    // Warning: there is an offset of 1 between the evaluation order saved
    // in the node and the actual location in the _variableOrder
    CPPADCG_ASSERT_UNKNOWN(fromPos > toPos)
    Node* node = _variableOrder[fromPos - 1]; // node to be moved

    // move variables in between the order change
    for (size_t l = fromPos - 1; l > toPos - 1; --l) {
        _variableOrder[l] = _variableOrder[l - 1];
        updateEvaluationQueueOrder(*_variableOrder[l], l + 1);
    }

    _variableOrder[toPos - 1] = node;
    updateEvaluationQueueOrder(*node, toPos);
}

template<class Base>
inline void CodeHandler<Base>::determineLastTempVarUsage(Node& root) {

    auto startAnalysis = [this](OperationStackData<Base>& stackEl,
                                OperationStack<Base>& stack) {
        auto& node = stackEl.node();
        CGOpCode op = node.getOperationType();

        markVisited(node);

        if (op == CGOpCode::LoopEnd) {
            auto& loopEnd = static_cast<LoopEndOperationNode<Base>&> (node);
            _loops.depth++;
            _loops.outerVars.resize(_loops.depth + 1);
            _loops.startEvalOrder.push_back(getEvaluationOrder(loopEnd.getLoopStart()));

        } else if (op == CGOpCode::LoopStart) {
            _loops.depth--; // leaving the current loop
        }

        /**
         * count variable usage
         */
        auto& args = node.getArguments();
        for (size_t i = 0; i < args.size(); ++i) {
            if (args[i].getOperation() != nullptr) {
                Node& arg = *args[i].getOperation();

                if (!isVisited(arg)) {
                    // dependencies not visited yet
                    stack.emplace_back(node, i, 0);
                }
            }
        }

        return true;
    };

    auto endAnalysis = [this](OperationStackData<Base>& stackEl) {
        // executed after all children have been visited

        auto& node = stackEl.node();
        CGOpCode op = node.getOperationType();

        for (const Arg& it : node.getArguments()) {
            if (it.getOperation() != nullptr) {
                Node& arg = *it.getOperation();

                size_t order = getEvaluationOrder(node);
                Node* aa = getOperationFromAlias(arg); // follow alias!
                if (aa != nullptr) {
                    if (getLastUsageEvaluationOrder(*aa) < order) {
                        setLastUsageEvaluationOrder(*aa, order);
                    }

                    if (_loops.depth >= 0 &&
                        getEvaluationOrder(*aa) < _loops.startEvalOrder[_loops.depth] &&
                        isTemporary(*aa)) {
                        // outer variable used inside the loop
                        _loops.outerVars[_loops.depth].insert(aa);
                    }
                }
            }
        }

        if (op == CGOpCode::LoopEnd) {
            /**
             * temporary variables from outside the loop which are used
             * within the loop cannot be overwritten inside that loop
             */
            size_t order = getEvaluationOrder(node);

            const std::set<Node*>& outerLoopUsages = _loops.outerVars.back();
            for (Node* outerVar : outerLoopUsages) {
                Node* aa = getOperationFromAlias(*outerVar); // follow alias!
                if (aa != nullptr && getLastUsageEvaluationOrder(*aa) < order)
                    setLastUsageEvaluationOrder(*aa, order);
            }

            _loops.depth--;
            _loops.outerVars.pop_back();
            _loops.startEvalOrder.pop_back();

        } else if (op == CGOpCode::LoopStart) {
            _loops.depth++; // coming back to the loop
        }

    };

    depthFirstGraphNavigation(root,
                              0,
                              startAnalysis,
                              endAnalysis,
                              true);

}

template<class Base>
inline void CodeHandler<Base>::dependentAdded2EvaluationQueue(Node& root) {

    auto analyse = [this](SimpleOperationStackData<Base>& stackEl,
                          SimpleOperationStack<Base>& stack) {
        auto& node = stackEl.parent();
        auto& arg = stackEl.node();

        if (getEvaluationOrder(arg) == 0) {
            setEvaluationOrder(arg, getEvaluationOrder(node));

            stack.pushNodeArguments(arg);
        }
    };

    depthFirstGraphNavigation(root, analyse, false);

}

template<class Base>
inline void CodeHandler<Base>::updateEvaluationQueueOrder(Node& root,
                                                          size_t newEvalOrder) {

    auto analyse = [&](SimpleOperationStackData<Base>& stackEl,
                       SimpleOperationStack<Base>& stack) {
        auto& node = stackEl.node();
        size_t oldEvalOrder = getEvaluationOrder(node);

        setEvaluationOrder(node, newEvalOrder);

        for (const Arg& a : node.getArguments()) {
            if (a.getOperation() != nullptr) {
                Node& arg = *a.getOperation();
                if (getEvaluationOrder(arg) == oldEvalOrder)
                    stack.pushNodeArguments(arg);
            }
        }
    };

    depthFirstGraphNavigation(root, analyse, true);
}

template<class Base>
inline void CodeHandler<Base>::findVariableDependencies() {
    _variableDependencies.resize(_variableOrder.size());

    for (size_t i = 0; i < _variableOrder.size(); i++) {
        Node& var = *_variableOrder[i];

        _variableDependencies[i].clear();
        startNewOperationTreeVisit();

        for (const auto& a : var) {
            if (a.getOperation() != nullptr) {
                findVariableDependencies(i, *a.getOperation());
            }
        }
    }
}

template<class Base>
inline void CodeHandler<Base>::findVariableDependencies(size_t i,
                                                        Node& root) {

    auto analyse = [&](SimpleOperationStackData<Base>& stackEl,
                       SimpleOperationStack<Base>& stack) {
        auto& node = stackEl.node();

        if (!isVisited(node)) {
            markVisited(node);

            if(_varId[node] != 0) {
                _variableDependencies[i].insert(&node);
            } else {
                stack.pushNodeArguments(node);
            }
        }
    };

    depthFirstGraphNavigation(root, analyse, true);
}

template<class Base>
inline bool CodeHandler<Base>::isIndependent(const Node& arg) const {
    return arg.getOperationType() == CGOpCode::Inv;
}

template<class Base>
inline bool CodeHandler<Base>::isTemporary(const Node& arg) const {
    CGOpCode op = arg.getOperationType();
    return op != CGOpCode::ArrayCreation && // classified as TemporaryArray
           op != CGOpCode::SparseArrayCreation && // classified as TemporarySparseArray
           op != CGOpCode::AtomicForward &&
           op != CGOpCode::AtomicReverse &&
           op != CGOpCode::LoopStart &&
           op != CGOpCode::LoopEnd &&
           op != CGOpCode::StartIf &&
           op != CGOpCode::ElseIf &&
           op != CGOpCode::Else &&
           op != CGOpCode::EndIf &&
           op != CGOpCode::LoopIndexedDep &&
           op != CGOpCode::LoopIndexedIndep &&
           op != CGOpCode::LoopIndexedTmp && // not considered as a temporary (the temporary is CGTmpDclOp)
           op != CGOpCode::Index &&
           op != CGOpCode::IndexAssign &&
           op != CGOpCode::Tmp && // not considered as a temporary (the temporary is CGTmpDclOp)
           _varId[arg] >= _minTemporaryVarID;
}

template<class Base>
inline bool CodeHandler<Base>::isTemporaryArray(const Node& arg) {
    return arg.getOperationType() == CGOpCode::ArrayCreation;
}

template<class Base>
inline bool CodeHandler<Base>::isTemporarySparseArray(const Node& arg) {
    return arg.getOperationType() == CGOpCode::SparseArrayCreation;
}

template<class Base>
inline OperationNode<Base>* CodeHandler<Base>::getOperationFromAlias(Node& alias) {
    if (alias.getOperationType() != CGOpCode::Alias) {
        return &alias;
    } else {
        Node* aa = &alias;
        do {
            CPPADCG_ASSERT_UNKNOWN(aa->getArguments().size() == 1)
            aa = aa->getArguments()[0].getOperation();
        } while (aa != nullptr && aa->getOperationType() == CGOpCode::Alias);
        return aa;
    }
}

template<class Base>
inline size_t CodeHandler<Base>::getEvaluationOrder(const Node& node) const {
    return _evaluationOrder[node];
}

template<class Base>
inline void CodeHandler<Base>::setEvaluationOrder(Node& node,
                                                  size_t order) {
    CPPADCG_ASSERT_UNKNOWN(order <= _variableOrder.size())
    _evaluationOrder[node] = order;
}

template<class Base>
inline size_t CodeHandler<Base>::getLastUsageEvaluationOrder(const Node& node) const {
    return _lastUsageOrder[node];
}

template<class Base>
inline void CodeHandler<Base>::setLastUsageEvaluationOrder(const Node& node,
                                                           size_t last) {
    CPPADCG_ASSERT_UNKNOWN(last <= _variableOrder.size()) // _lastUsageOrder[node] = 0  means that it was never used
    _lastUsageOrder[node] = last;

    CGOpCode op = node.getOperationType();
    if (op == CGOpCode::ArrayElement) {
        Node* array = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_UNKNOWN(array->getOperationType() == CGOpCode::ArrayCreation)
        if (getLastUsageEvaluationOrder(*array) < last) {
            setLastUsageEvaluationOrder(*array, last);
        }
    } else if (op == CGOpCode::Tmp) {
        Node* declr = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_UNKNOWN(declr->getOperationType() == CGOpCode::TmpDcl)
        if (getLastUsageEvaluationOrder(*declr) < last) {
            setLastUsageEvaluationOrder(*declr, last);
        }
    }
}

template<class Base>
inline size_t CodeHandler<Base>::getTotalUsageCount(const Node& node) const {
    return _totalUseCount[node];
}

template<class Base>
inline void CodeHandler<Base>::setTotalUsageCount(const Node& node,
                                                  size_t cout) {
    _totalUseCount[node] = cout;
}

template<class Base>
inline void CodeHandler<Base>::increaseTotalUsageCount(const Node& node) {
    _totalUseCount[node]++;
}

template<class Base>
inline void CodeHandler<Base>::resetManagedNodes() {
    _variableOrder.clear();
    _scopedVariableOrder.resize(1);
    _scopedVariableOrder[0].clear();
    _evaluationOrder.fill(0);
    _lastUsageOrder.fill(0);
    _totalUseCount.fill(0);
    _operationCount.fill(0);
    _varId.fill(0);
    _scope.fill(0);
}

} // END cg namespace
} // END CppAD namespace

#endif
