#ifndef CPPAD_CG_LANGUAGE_DOT_INCLUDED
#define CPPAD_CG_LANGUAGE_DOT_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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
 * Generates the model using the dot language used by graphviz.
 * 
 * @author Joao Leal
 */
template<class Base>
class LanguageDot : public Language<Base> {
protected:
    static const std::string _C_STATIC_INDEX_ARRAY;
    static const std::string _C_SPARSE_INDEX_ARRAY;
protected:
    // information from the code handler (not owned)
    LanguageGenerationData <Base>* _info;
    // new line characters
    std::string _endline;
    // output stream for the generated source code
    std::ostringstream _code;
    // creates the variable names
    VariableNameGenerator <Base>* _nameGen;
    // auxiliary string stream
    std::ostringstream _ss;
    //
    size_t _independentSize;
    //
    size_t _minTemporaryVarID;
    // maps the variable IDs to the their position in the dependent vector
    // (some IDs may be the same as the independent variables when dep = indep)
    std::map<size_t, size_t> _dependentIDs;
    // the dependent variable vector
    const ArrayView<CG<Base> >* _dependent;
    // whether or not to ignore assignment of constant zero values to dependent variables
    bool _ignoreZeroDepAssign;
    // the name of the file to be created without the extension
    std::string _filename;
    //
    std::vector<const LoopStartOperationNode <Base>*> _currentLoops;
    // the maximum precision used to print values
    size_t _parameterPrecision;
    //
    bool _combineParameterNodes;
    //
    std::string _indepNodeStyle;
    //
    std::string _depNodeStyle;
private:
    std::vector<int> varIds_;
    size_t parIdx_;
public:

    /**
     * Creates a MathML language source code generator
     */
    LanguageDot() :
            _info(nullptr),
            _endline("\n"),
            _nameGen(nullptr),
            _independentSize(0), // not really required (but it avoids warnings)
            _minTemporaryVarID(0), // not really required (but it avoids warnings)
            _dependent(nullptr),
            _ignoreZeroDepAssign(false),
            _filename("algorithm"),
            _parameterPrecision(std::numeric_limits<Base>::digits10),
            _combineParameterNodes(true),
            parIdx_(0) { // not really required (but it avoids warnings)
    }

    inline virtual ~LanguageDot() {
    }

    inline bool isIgnoreZeroDepAssign() const {
        return _ignoreZeroDepAssign;
    }

    inline void setIgnoreZeroDepAssign(bool ignore) {
        _ignoreZeroDepAssign = ignore;
    }

    inline void setFilename(const std::string& name) {
        _filename = name;
    }

    /**
     * Provides the maximum precision used to print constant values in the
     * generated source code
     * 
     * @return the maximum number of digits
     */
    virtual size_t getParameterPrecision() const {
        return _parameterPrecision;
    }

    /**
     * Defines the maximum precision used to print constant values in the
     * generated source code
     * 
     * @param p the maximum number of digits
     */
    virtual void setParameterPrecision(size_t p) {
        _parameterPrecision = p;
    }

    /**
     * Defines the style for the independent variable nodes
     */
    inline void setIndepNodeStyle(const std::string& indepNodeStyle) {
        _indepNodeStyle = indepNodeStyle;
    }

    /**
     * Provides the style for the independent variable nodes
     */
    inline const std::string& getIndepNodeStyle() const {
        return _indepNodeStyle;
    }

    /**
     * Defines the style for the dependent variable nodes
     */
    inline void setDepNodeStyle(const std::string& depNodeStyle) {
        _depNodeStyle = depNodeStyle;
    }

    /**
     * Provides the style for the dependent variable nodes
     */
    inline const std::string& getDepNodeStyle() const {
        return _depNodeStyle;
    }

    /**
     * 
     */
    inline void setCombineParameterNodes(bool combineParameterNodes) {
        _combineParameterNodes = combineParameterNodes;
    }
    
    /**
     * 
     */
    inline bool isCombineParameterNodes() const {
        return _combineParameterNodes;
    }

    /***************************************************************************
     *                               STATIC
     **************************************************************************/
    static inline void printIndexCondExpr(std::ostringstream& out,
                                          const std::vector<size_t>& info,
                                          const std::string& index) {
        CPPADCG_ASSERT_KNOWN(info.size() > 1 && info.size() % 2 == 0, "Invalid number of information elements for an index condition expression operation")

        size_t infoSize = info.size();
        for (size_t e = 0; e < infoSize; e += 2) {
            if (e > 0) {
                out << " or "; // or
            }
            size_t min = info[e];
            size_t max = info[e + 1];
            if (min == max) {
                out << index << "==" << min;
            } else if (min == 0) {
                out << index << "≤" << max;
            } else if (max == (std::numeric_limits<size_t>::max)()) {
                out << min << "≤" << index;
            } else {
                if (infoSize != 2)
                    out << "(";

                if (max - min == 1)
                    out << min << "==" << index << " or " << index << "==" << max;
                else
                    out << min << "≤" << index << " and " << index << "≤" << max;

                if (infoSize != 2)
                    out << ")";
            }
        }
    }

    /***************************************************************************
     * 
     **************************************************************************/

    inline void printStaticIndexArray(std::ostringstream& os,
                                      const std::string& name,
                                      const std::vector<size_t>& values);

    inline void printStaticIndexMatrix(std::ostringstream& os,
                                       const std::string& name,
                                       const std::map<size_t, std::map<size_t, size_t> >& values);

    /***************************************************************************
     * index patterns
     **************************************************************************/
    static inline void generateNames4RandomIndexPatterns(const std::set<RandomIndexPattern*>& randomPatterns);

    inline void printRandomIndexPatternDeclaration(std::ostringstream& os,
                                                   const std::set<RandomIndexPattern*>& randomPatterns);

    static void indexPattern2String(std::ostream& os,
                                    const IndexPattern& ip,
                                    const OperationNode<Base>& index);

    static void indexPattern2String(std::ostream& os,
                                    const IndexPattern& ip,
                                    const std::vector<const OperationNode<Base>*>& indexes);

    static inline void linearIndexPattern2String(std::ostream& os,
                                                 const LinearIndexPattern& lip,
                                                 const OperationNode<Base>& index);

    /***************************************************************************
     *                              protected
     **************************************************************************/
protected:

    void generateSourceCode(std::ostream& out,
                            std::unique_ptr<LanguageGenerationData<Base> > info) override {
        // clean up
        _code.str("");
        _ss.str("");
        _currentLoops.clear();
        _dependentIDs.clear();
        parIdx_ = 0;

        // save some info
        _info = info.get();
        _independentSize = info->independent.size();
        _dependent = &info->dependent;
        _nameGen = &info->nameGen;
        _minTemporaryVarID = info->minTemporaryVarID;
        const ArrayView<CG<Base> >& dependent = info->dependent;
        const std::vector<OperationNode<Base>*>& variableOrder = info->variableOrder;

        varIds_.resize(_minTemporaryVarID + variableOrder.size());
        std::fill(varIds_.begin(), varIds_.end(), 0);

        _code << "digraph {" << _endline << _endline;

        /**
         * generate index array names (might be used for variable names)
         */
        generateNames4RandomIndexPatterns(info->indexRandomPatterns);

        /**
         * generate variable names
         */
        //generate names for the independent variables
        _code << "subgraph indep {" << _endline;
        _code << "   rank=min" << _endline;
        if(!_indepNodeStyle.empty()) {
            _code << "node [" << _indepNodeStyle << "]" << _endline;
        }
        for (size_t j = 0; j < _independentSize; j++) {
            OperationNode<Base>& op = *info->independent[j];

            _code << "   v" << op.getHandlerPosition() << " [label=\"";
            if (op.getName() == nullptr) {
                _code << _nameGen->generateIndependent(op, getVariableID(op));
            } else {
                _code << *op.getName();
            }
            _code << "\"]" << _endline;

        }
        _code << "}" << _endline;

        // generate names for the dependent variables (must be after naming independents)
        _code << "subgraph dep {" << _endline;
        _code << "   rank=max" << _endline;
        if(!_depNodeStyle.empty()) {
            _code << "node [" << _depNodeStyle << "]" << _endline;
        }
        for (size_t i = 0; i < dependent.size(); i++) {

            OperationNode<Base>* node = dependent[i].getOperationNode();
            if (node != nullptr && node->getOperationType() != CGOpCode::LoopEnd) {
                _code << "   y" << i << " [label=\"";
                if(node->getName() == nullptr) {
                    if (node->getOperationType() == CGOpCode::LoopIndexedDep) {
                        size_t pos = node->getInfo()[0];
                        const IndexPattern* ip = info->loopDependentIndexPatterns[pos];
                        _code << _nameGen->generateIndexedDependent(*node, getVariableID(*node), *ip);

                    } else {
                        _code << _nameGen->generateDependent(i);
                    }
                } else {
                    _code << *node->getName();
                }
                _code << "\"]" << _endline;
            }
        }
        _code << "}" << _endline;


        /**
         * Loop indexes
         */
        printRandomIndexPatternDeclaration(_ss, _info->indexRandomPatterns);

        /**
         * function variable declaration
         */
        const std::vector<FuncArgument>& indArg = _nameGen->getIndependent();
        const std::vector<FuncArgument>& depArg = _nameGen->getDependent();
        const std::vector<FuncArgument>& tmpArg = _nameGen->getTemporary();
        CPPADCG_ASSERT_KNOWN(indArg.size() > 0 && !depArg.empty(),
                             "There must be at least one dependent and one independent argument")
        CPPADCG_ASSERT_KNOWN(tmpArg.size() == 3,
                             "There must be three temporary variables")

        /**
         * Determine the dependent variables that result from the same operations
         */
        // dependent variables indexes that are copies of other dependent variables
        std::set<size_t> dependentDuplicates;

        for (size_t i = 0; i < dependent.size(); i++) {
            OperationNode<Base>* node = dependent[i].getOperationNode();
            if (node != nullptr) {
                CGOpCode type = node->getOperationType();
                if (type != CGOpCode::Inv && type != CGOpCode::LoopEnd) {
                    size_t varID = getVariableID(*node);
                    if (varID > 0) {
                        auto it2 = _dependentIDs.find(varID);
                        if (it2 == _dependentIDs.end()) {
                            _dependentIDs[getVariableID(*node)] = i;
                        } else {
                            // there can be several dependent variables with the same ID
                            dependentDuplicates.insert(i);
                        }
                    }
                }
            }
        }

        /**
         * non-constant variables
         */
        if (variableOrder.size() > 0) {
            // generate names for temporary variables
            for (OperationNode<Base>* node : variableOrder) {
                CGOpCode op = node->getOperationType();
                if (!isDependent(*node) && op != CGOpCode::IndexDeclaration) {
                    // variable names for temporaries must always be created since they might have been used before with a different name/id
                    if (requiresVariableName(*node) && op != CGOpCode::ArrayCreation && op != CGOpCode::SparseArrayCreation) {
                        node->setName(_nameGen->generateTemporary(*node, getVariableID(*node)));
                    } else if (op == CGOpCode::ArrayCreation) {
                        node->setName(_nameGen->generateTemporaryArray(*node, getVariableID(*node)));
                    } else if (op == CGOpCode::SparseArrayCreation) {
                        node->setName(_nameGen->generateTemporarySparseArray(*node, getVariableID(*node)));
                    }
                }
            }

            /**
             * Source code generation magic!
             */
            for (OperationNode<Base>* it : variableOrder) {
                // check if a new function should start
                OperationNode<Base>& node = *it;

                // a dependent variable assigned by a loop does require any source code (its done inside the loop)
                if (node.getOperationType() == CGOpCode::DependentRefRhs) {
                    continue; // nothing to do (this operation is right hand side only)
                } else if (node.getOperationType() == CGOpCode::TmpDcl) { // temporary variable declaration does not need any source code here
                    continue; // nothing to do (bogus operation)
                }

                printExpressionNoVarCheck(node);
            }

        }

        // dependent duplicates
        if (!dependentDuplicates.empty()) {
            _code << "// variable duplicates: " << dependentDuplicates.size() << _endline;

            for (size_t index : dependentDuplicates) {
                const CG<Base>& dep = dependent[index];
                OperationNode<Base>* depNode = dep.getOperationNode();

                _code << makeNodeName(*depNode);
                _code << " -> y" << index;
                _code << _endline;
            }
        }

        for (size_t i = 0; i < dependent.size(); i++) {
            if (!dependent[i].isParameter() && dependent[i].getOperationNode()->getOperationType() != CGOpCode::Inv) {
                _code << makeNodeName(*dependent[i].getOperationNode());
                _code << " -> y" << i;
                _code << _endline;
            }
        }

        // constant dependent variables 
        bool commentWritten = false;
        for (size_t i = 0; i < dependent.size(); i++) {
            if (dependent[i].isParameter()) {
                if (!_ignoreZeroDepAssign || !dependent[i].isIdenticalZero()) {
                    if (!commentWritten) {
                        _code << "// dependent variables without operations" << _endline;
                        commentWritten = true;
                    }

                    _code << makeNodeName(dependent[i].getValue());
                    _code << " -> y" << i;
                    _code << _endline;
                }
            } else if (dependent[i].getOperationNode()->getOperationType() == CGOpCode::Inv) {
                if (!commentWritten) {
                    _code << "// dependent variables without operations" << _endline;
                    commentWritten = true;
                }

                _code << makeNodeName(*dependent[i].getOperationNode());
                _code << " -> y" << i;
                _code << _endline;
            }
        }

        _code << _endline << "}" << _endline; // digraph

        // a single source file
        out << _code.str();
    }

    inline size_t getVariableID(const OperationNode<Base>& node) const {
        return _info->varId[node];
    }

    bool createsNewVariable(const OperationNode<Base>& var,
                            size_t totalUseCount,
                            size_t opCount) const override {
        CGOpCode op = var.getOperationType();
        if (totalUseCount > 1) {
            return op != CGOpCode::ArrayElement && op != CGOpCode::Index && op != CGOpCode::IndexDeclaration && op != CGOpCode::Tmp;
        } else {
            return (op == CGOpCode::ArrayCreation ||
                    op == CGOpCode::SparseArrayCreation ||
                    op == CGOpCode::AtomicForward ||
                    op == CGOpCode::AtomicReverse ||
                    op == CGOpCode::ComLt ||
                    op == CGOpCode::ComLe ||
                    op == CGOpCode::ComEq ||
                    op == CGOpCode::ComGe ||
                    op == CGOpCode::ComGt ||
                    op == CGOpCode::ComNe ||
                    op == CGOpCode::LoopIndexedDep ||
                    op == CGOpCode::LoopIndexedTmp ||
                    op == CGOpCode::IndexAssign ||
                    op == CGOpCode::Assign) &&
                   op != CGOpCode::CondResult;
        }
    }

    virtual bool requiresVariableName(const OperationNode<Base>& var) const {
        CGOpCode op = var.getOperationType();
        return (_info->totalUseCount.get(var) > 1 &&
                op != CGOpCode::AtomicForward &&
                op != CGOpCode::AtomicReverse &&
                op != CGOpCode::LoopStart &&
                op != CGOpCode::LoopEnd &&
                op != CGOpCode::Index &&
                op != CGOpCode::IndexAssign &&
                op != CGOpCode::StartIf &&
                op != CGOpCode::ElseIf &&
                op != CGOpCode::Else &&
                op != CGOpCode::EndIf &&
                op != CGOpCode::CondResult &&
                op != CGOpCode::LoopIndexedTmp &&
                op != CGOpCode::Tmp);
    }

    /**
     * Whether or not this operation assign its expression to a variable by
     * itself.
     * 
     * @param var the operation node
     * @return 
     */
    virtual bool directlyAssignsVariable(const OperationNode<Base>& var) const {
        CGOpCode op = var.getOperationType();
        return isCondAssign(op) ||
               op == CGOpCode::ArrayCreation ||
               op == CGOpCode::SparseArrayCreation ||
               op == CGOpCode::AtomicForward ||
               op == CGOpCode::AtomicReverse ||
               op == CGOpCode::DependentMultiAssign ||
               op == CGOpCode::LoopStart ||
               op == CGOpCode::LoopEnd ||
               op == CGOpCode::IndexAssign ||
               op == CGOpCode::StartIf ||
               op == CGOpCode::ElseIf ||
               op == CGOpCode::Else ||
               op == CGOpCode::EndIf ||
               op == CGOpCode::CondResult ||
               op == CGOpCode::IndexDeclaration;
    }

    bool requiresVariableArgument(enum CGOpCode op, size_t argIndex) const override {
        return op == CGOpCode::CondResult;
    }

    inline const std::string& createVariableName(OperationNode<Base>& var) {
        CGOpCode op = var.getOperationType();
        CPPADCG_ASSERT_UNKNOWN(getVariableID(var) > 0)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::AtomicForward)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::AtomicReverse)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::LoopStart)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::LoopEnd)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::Index)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::IndexAssign)
        CPPADCG_ASSERT_UNKNOWN(op != CGOpCode::IndexDeclaration)

        if (var.getName() == nullptr) {
            if (op == CGOpCode::ArrayCreation) {
                var.setName(_nameGen->generateTemporaryArray(var, getVariableID(var)));

            } else if (op == CGOpCode::SparseArrayCreation) {
                var.setName(_nameGen->generateTemporarySparseArray(var, getVariableID(var)));

            } else if (op == CGOpCode::LoopIndexedDep) {
                size_t pos = var.getInfo()[0];
                const IndexPattern* ip = _info->loopDependentIndexPatterns[pos];
                var.setName(_nameGen->generateIndexedDependent(var, getVariableID(var), *ip));

            } else if (op == CGOpCode::LoopIndexedIndep) {
                size_t pos = var.getInfo()[1];
                const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
                var.setName(_nameGen->generateIndexedIndependent(var, getVariableID(var), *ip));

            } else if (getVariableID(var) <= _independentSize) {
                // independent variable
                var.setName(_nameGen->generateIndependent(var, getVariableID(var)));

            } else if (getVariableID(var) < _minTemporaryVarID) {
                // dependent variable
                auto it = _dependentIDs.find(getVariableID(var));
                CPPADCG_ASSERT_UNKNOWN(it != _dependentIDs.end());

                size_t index = it->second;
                var.setName(_nameGen->generateDependent(index));

            } else if (op == CGOpCode::LoopIndexedTmp || op == CGOpCode::Tmp) {
                CPPADCG_ASSERT_KNOWN(var.getArguments().size() >= 1, "Invalid number of arguments for loop indexed temporary operation")
                OperationNode<Base>* tmpVar = var.getArguments()[0].getOperation();
                CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation");
                return createVariableName(*tmpVar);

            } else {
                // temporary variable
                var.setName(_nameGen->generateTemporary(var, getVariableID(var)));
            }
        }

        return *var.getName();
    }

    bool requiresVariableDependencies() const override {
        return false;
    }

    virtual std::string print(const Argument<Base>& arg) {
        if (arg.getOperation() != nullptr) {
            // expression
            return printExpression(*arg.getOperation());
        } else {
            // parameter
            return printParameter(*arg.getParameter());
        }
    }

    virtual std::string printExpression(OperationNode<Base>& node) {
        if (getVariableID(node) == 0) {
            // print expression code
            return printExpressionNoVarCheck(node);
        } else {
            return makeNodeName(node);
        }
    }

    virtual std::string printParameter(const Base& value) {
        if(!_combineParameterNodes) {
            std::string name = makeNodeName(value);

            _code << name;
            _code << " [label=\"";
            _code << std::setprecision(_parameterPrecision) << value;
            _code << "\"]" << _endline;

            return name;
        } else {
            return makeNodeName(value);
        }
    }

    inline virtual std::string makeNodeName(const OperationNode<Base>& node) {
        return "v" + std::to_string(node.getHandlerPosition());
    }

    inline std::string makeNodeName(const Argument<Base>& arg) {
        if (arg.getOperation() != nullptr) {
            // expression
            return makeNodeName(*arg.getOperation());
        } else {
            // parameter
            return makeNodeName(*arg.getParameter());
        }
    }

    inline virtual std::string makeNodeName(const Base& value) {
        if(_combineParameterNodes) {
            // node name for parameters which have the same node for the same value
            _ss.str("");
            _ss << "\"" << std::setprecision(_parameterPrecision) << value << "\"";
            return _ss.str();
        } else {
            std::string name = "p" + std::to_string(parIdx_);
            parIdx_++;
            return name;
        }
    }

    inline std::string printNodeDeclaration(const OperationNode<Base>& op,
                                            const std::ostringstream& label,
                                            const std::string& shape = "") {
        return printNodeDeclaration(op, label.str(), shape);
    }

    virtual std::string printNodeDeclaration(const OperationNode<Base>& op,
                                             const std::string& label = "",
                                             const std::string& shape = "") {
        std::string name = makeNodeName(op);

        _code << name << " [label=\"";
        if(!label.empty()) {
            _code << label;
        } else {
            _code << op.getOperationType();
        }
        _code << "\"";
        if (!shape.empty()) {
            _code << ", shape=" << shape;
        }
        _code << "]" << _endline;

        return name;
    }

    inline void printEdges(const std::string& name,
                           const OperationNode<Base>& node,
                           const std::string& style = "") {
        const auto& args = node.getArguments();

        std::vector<std::string> aNames(args.size());
        for (size_t i = 0; i < args.size(); ++i) {
            aNames[i] = print(args[i]);
        }

        for (size_t i = 0; i < args.size(); ++i) {
            if (i > 0)
                _code << "  ";
            printEdge(aNames[i], name, style);
        }
        _code << _endline;
    }

    inline void printEdges(const std::string& name,
                           const OperationNode<Base>& node,
                           const std::vector<std::string>& args,
                           const std::string& style = "") {
        size_t na = node.getArguments().size();
        size_t nna = args.size();
        CPPADCG_ASSERT_UNKNOWN(na >= nna)

        for (size_t i = 0; i < na; ++i) {
            if (i > 0)
                _code << "  ";
            if(i < nna && !args[i].empty()) {
                printEdge(args[i], name, style);
            } else {
                std::string n = print(node.getArguments()[i]);
                printEdge(n, name, style);
            }
        }
        _code << _endline;
    }

    inline void printEdges(const std::string& name,
                           const OperationNode<Base>& node,
                           const std::vector<std::string>& args,
                           const std::vector<std::string>& styles) {
        size_t na = node.getArguments().size();
        size_t nna = args.size();
        size_t ns = styles.size();
        CPPADCG_ASSERT_UNKNOWN(na >= nna);
        CPPADCG_ASSERT_UNKNOWN(na >= ns);

        std::string style;
        for (size_t i = 0; i < args.size(); ++i) {
            if (i > 0)
                _code << "  ";

            style = i < ns ? styles[i] : "";
            if(i < nna && !args[i].empty()) {
                printEdge(args[i], name, style);
            } else {
                std::string n = print(node.getArguments()[i]);
                printEdge(n, name, style);
            }
        }
        _code << _endline;
    }

    inline void printEdge(const OperationNode<Base>& from,
                          const std::string& to,
                          const std::string& style = "") {
        _code << makeNodeName(from);
        _code << " -> " << to;
        if (!style.empty())
            _code << "[" << style << "]";
    }

    inline void printEdge(const std::string& from,
                          const std::string& to,
                          const std::string& style = "") {
        _code << from << " -> " << to;
        if (!style.empty())
            _code << "[" << style << "]";
    }

    virtual std::string printExpressionNoVarCheck(OperationNode<Base>& node) {
        CGOpCode op = node.getOperationType();
        switch (op) {
            case CGOpCode::ArrayCreation:
                return printArrayCreationOp(node);
            case CGOpCode::SparseArrayCreation:
                return printSparseArrayCreationOp(node);
            case CGOpCode::ArrayElement:
                return printArrayElementOp(node);
            case CGOpCode::Assign:
                return printAssignOp(node);

            case CGOpCode::Abs:
            case CGOpCode::Acos:
            case CGOpCode::Asin:
            case CGOpCode::Atan:
            case CGOpCode::Cosh:
            case CGOpCode::Cos:
            case CGOpCode::Exp:
            case CGOpCode::Log:
            case CGOpCode::Sign:
            case CGOpCode::Sinh:
            case CGOpCode::Sin:
            case CGOpCode::Sqrt:
            case CGOpCode::Tanh:
            case CGOpCode::Tan:
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
            case CGOpCode::Erfc:
            case CGOpCode::Asinh:
            case CGOpCode::Acosh:
            case CGOpCode::Atanh:
            case CGOpCode::Expm1:
            case CGOpCode::Log1p:
#endif
                return printUnaryFunction(node);
            case CGOpCode::AtomicForward: // atomicFunction.forward(q, p, vx, vy, tx, ty)
                return printAtomicForwardOp(node);
            case CGOpCode::AtomicReverse: // atomicFunction.reverse(p, tx, ty, px, py)
                return printAtomicReverseOp(node);
            case CGOpCode::Add:
                return printOperationAdd(node);
            case CGOpCode::Alias:
                return printOperationAlias(node);
            case CGOpCode::ComLt:
            case CGOpCode::ComLe:
            case CGOpCode::ComEq:
            case CGOpCode::ComGe:
            case CGOpCode::ComGt:
            case CGOpCode::ComNe:
                return printConditionalAssignment(node);
            case CGOpCode::Div:
                return printOperationDiv(node);
            case CGOpCode::Inv:
                // do nothing
                return makeNodeName(node);
            case CGOpCode::Mul:
                return printOperationMul(node);
            case CGOpCode::Pow:
                return printPowFunction(node);
            case CGOpCode::Pri:
                // do nothing
                return makeNodeName(node);
            case CGOpCode::Sub:
                return printOperationMinus(node);

            case CGOpCode::UnMinus:
                return printOperationUnaryMinus(node);

            case CGOpCode::DependentMultiAssign:
                return printDependentMultiAssign(node);

            case CGOpCode::Index:
                return makeNodeName(node); // nothing to do

            case CGOpCode::IndexAssign:
                return printIndexAssign(node);
            case CGOpCode::IndexDeclaration:
                return makeNodeName(node); // already done

            case CGOpCode::LoopStart:
                return printLoopStart(node);
            case CGOpCode::LoopIndexedIndep:
                return printLoopIndexedIndep(node);
            case CGOpCode::LoopIndexedDep:
                return printLoopIndexedDep(node);
            case CGOpCode::LoopIndexedTmp:
                return printLoopIndexedTmp(node);
            case CGOpCode::TmpDcl:
                // nothing to do
                return makeNodeName(node);
            case CGOpCode::Tmp:
                return printTmpVar(node);
            case CGOpCode::LoopEnd:
                return printLoopEnd(node);
            case CGOpCode::IndexCondExpr:
                return printIndexCondExprOp(node);
            case CGOpCode::StartIf:
                return printStartIf(node);
            case CGOpCode::ElseIf:
                return printElseIf(node);
            case CGOpCode::Else:
                return printElse(node);
            case CGOpCode::EndIf:
                return printEndIf(node);
            case CGOpCode::CondResult:
                return printCondResult(node);
            default:
                throw CGException("Unknown operation code '", op, "'.");
        }
    }

    virtual std::string printAssignOp(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for assign operation")

        return print(node.getArguments()[0]);
    }

    virtual std::string printPowFunction(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for pow() function")

        std::string a0 = print(op.getArguments()[0]);
        std::string a1 = print(op.getArguments()[1]);

        std::string name = printNodeDeclaration(op);

        printEdges(name, op, std::vector<std::string>{a0, a1}, std::vector<std::string>{"label=\"$1\"", "label=\"$2\""});

        return name;
    }

    virtual std::string printUnaryFunction(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for an unary function")

        std::string a0 = print(op.getArguments()[0]);

        // TODO: improve this
        _ss.str("");
        _ss << op.getOperationType();
        std::string label = _ss.str();
        auto it = label.find('(');
        if (it != std::string::npos) {
            label = label.substr(0, it);
        }
        std::string name = printNodeDeclaration(op, label);

        printEdges(name, op, std::vector<std::string> {a0});

        return name;
    }

    virtual std::string printOperationAlias(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for alias")

        std::string a0 = print(op.getArguments()[0]);

        std::string name = printNodeDeclaration(op);

        printEdges(name, op, std::vector<std::string> {a0});

        return name;
    }

    virtual std::string printOperationAdd(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for addition")

        const Argument<Base>& left = op.getArguments()[0];
        const Argument<Base>& right = op.getArguments()[1];

        std::string a0 = print(left);
        std::string a1 = print(right);

        std::string name = printNodeDeclaration(op, "+");

        printEdges(name, op, std::vector<std::string> {a0, a1});

        return name;
    }

    virtual std::string printOperationMinus(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for subtraction")

        const Argument<Base>& left = op.getArguments()[0];
        const Argument<Base>& right = op.getArguments()[1];

        std::string a0 = print(left);
        std::string a1 = print(right);

        std::string name = printNodeDeclaration(op);

        printEdges(name, op, std::vector<std::string> {a0, a1}, std::vector<std::string>{"label=\"$1\"", "label=\"$2\""});

        return name;
    }

    virtual std::string printOperationDiv(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for division")

        const Argument<Base>& left = op.getArguments()[0];
        const Argument<Base>& right = op.getArguments()[1];

        std::string a0 = print(left);
        std::string a1 = print(right);

        std::string name = printNodeDeclaration(op);

        printEdges(name, op, std::vector<std::string> {a0, a1}, std::vector<std::string>{"label=\"$1\"", "label=\"$2\""});

        return name;
    }

    virtual std::string printOperationMul(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for multiplication")

        const Argument<Base>& left = op.getArguments()[0];
        const Argument<Base>& right = op.getArguments()[1];

        std::string a0 = print(left);
        std::string a1 = print(right);

        std::string name = printNodeDeclaration(op, "×");

        printEdges(name, op, std::vector<std::string> {a0, a1});

        return name;
    }

    virtual std::string printOperationUnaryMinus(OperationNode<Base>& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary minus")

        const Argument<Base>& arg = op.getArguments()[0];

        std::string a0 = print(arg);

        std::string name = printNodeDeclaration(op);

        printEdges(name, op, std::vector<std::string> {a0});

        return name;
    }

    virtual std::string printConditionalAssignment(OperationNode<Base>& node) {
        CPPADCG_ASSERT_UNKNOWN(getVariableID(node) > 0)

        const std::vector<Argument<Base> >& args = node.getArguments();
        const Argument<Base>& left = args[0];
        const Argument<Base>& right = args[1];
        const Argument<Base>& trueCase = args[2];
        const Argument<Base>& falseCase = args[3];

        std::string a0 = print(left);
        std::string a1 = print(right);
        std::string a2 = print(trueCase);
        std::string a3 = print(falseCase);

        std::string name = printNodeDeclaration(node, "", "diamond");

        /**
         * Connections
         */
        printEdges(name, node, std::vector<std::string> {a0, a1, a2, a3},
                   std::vector<std::string>{"label=\"left\"", "label=\"right\"", "label=\"true\"", "label=\"false\""});

        return name;
    }

    virtual std::string printArrayCreationOp(OperationNode<Base>& op);

    virtual std::string printSparseArrayCreationOp(OperationNode<Base>& op);

    inline size_t printArrayCreationUsingLoop(const std::string& arrayName,
                                              const OperationNode<Base>& array,
                                              size_t startj,
                                              const size_t* indexes);

    virtual std::string printArrayElementOp(OperationNode<Base>& op);

    virtual std::string printAtomicForwardOp(OperationNode<Base>& atomicFor) {
        CPPADCG_ASSERT_KNOWN(atomicFor.getInfo().size() == 3, "Invalid number of information elements for atomic forward operation")
        int q = atomicFor.getInfo()[1];
        int p = atomicFor.getInfo()[2];
        size_t p1 = p + 1;
        const std::vector<Argument<Base> >& opArgs = atomicFor.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 2, "Invalid number of arguments for atomic forward operation")

        size_t id = atomicFor.getInfo()[0];

        std::string name = printNodeDeclaration(atomicFor, _info->atomicFunctionId2Name.at(id) + ".forward(" + std::to_string(q) + ", " + std::to_string(p) + ", tx, ty)");

        /**
         * Edges
         */
        std::vector<std::string> args(opArgs.size()); // argument node names

        std::vector<OperationNode<Base>*> tx(p1), ty(p1);
        for (size_t k = 0; k < p1; k++) {
            tx[k] = opArgs[0 * p1 + k].getOperation();
            ty[k] = opArgs[1 * p1 + k].getOperation();

            args[0 * p1 + k] = print(*tx[k]);
            args[1 * p1 + k] = print(*ty[k]);
        }

        for (size_t k = 0; k < p1; k++) {
            printEdge(args[0 * p1 + k], name, "label=\"tx" + std::to_string(k) + "\"");
            _code << "  ";

            printEdge(args[1 * p1 + k], name, "label=\"ty" + std::to_string(k) + "\"");
            _code << "  ";
        }
        _code << _endline;

        CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(ty[p]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")

        return name;
    }

    virtual std::string printAtomicReverseOp(OperationNode<Base>& atomicRev) {
        CPPADCG_ASSERT_KNOWN(atomicRev.getInfo().size() == 2, "Invalid number of information elements for atomic reverse operation")
        int p = atomicRev.getInfo()[1];
        size_t p1 = p + 1;
        const std::vector<Argument<Base> >& opArgs = atomicRev.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 4, "Invalid number of arguments for atomic reverse operation")

        size_t id = atomicRev.getInfo()[0];

        std::string name = printNodeDeclaration(atomicRev, _info->atomicFunctionId2Name.at(id) + ".reverse(" + std::to_string(p) + ", tx, px, py)");

        /**
         * Edges
         */
        std::vector<std::string> args(opArgs.size()); // argument node names

        std::vector<OperationNode<Base>*> tx(p1), px(p1), py(p1);
        for (size_t k = 0; k < p1; k++) {
            tx[k] = opArgs[0 * p1 + k].getOperation();
            px[k] = opArgs[2 * p1 + k].getOperation();
            py[k] = opArgs[3 * p1 + k].getOperation();

            args[0 * p1 + k] = print(*tx[k]);
            args[1 * p1 + k] = print(opArgs[1 * p1 + k]); // todo: consider not showing this
            args[2 * p1 + k] = print(*px[k]);
            args[3 * p1 + k] = print(*py[k]);
        }

        for (size_t k = 0; k < p1; k++) {
            printEdge(args[0 * p1 + k], name, "label=\"tx" + std::to_string(k) + "\"");
            _code << "  ";

            printEdge(args[1 * p1 + k], name, "label=\"ty" + std::to_string(k) + "\"");
            _code << "  ";

            printEdge(args[2 * p1 + k], name, "label=\"px" + std::to_string(k) + "\"");
            _code << "  ";

            printEdge(args[3 * p1 + k], name, "label=\"py" + std::to_string(k) + "\"");
            _code << "  ";
        }
        _code << _endline;

        CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type")

        CPPADCG_ASSERT_KNOWN(px[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")

        CPPADCG_ASSERT_KNOWN(py[0]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(p == 0 || py[1]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")

        return name;
    }

    virtual std::string printDependentMultiAssign(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::DependentMultiAssign, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments")

        std::string name = printNodeDeclaration(node, "+=");

        const std::vector<Argument<Base> >& args = node.getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            bool useArg = false;
            const Argument<Base>& arg = args[a];
            std::string aName = print(arg);

            if (arg.getParameter() != nullptr) {
                useArg = true;
            } else {
                CGOpCode op = arg.getOperation()->getOperationType();
                useArg = op != CGOpCode::DependentRefRhs && op != CGOpCode::LoopEnd && op != CGOpCode::EndIf;
            }

            if (useArg) {
                printEdge(aName, name, "label=\"+=\"");
                _code << _endline;
                break;
            } else {
                printEdge(aName, name, "color=grey");
                _code << _endline;
            }
        }

        return name;
    }

    virtual std::string printLoopStart(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopStart, "Invalid node type")

        auto& lnode = static_cast<LoopStartOperationNode<Base>&> (node);
        _currentLoops.push_back(&lnode);

        /**
         * declaration
         */
        const std::string& jj = *lnode.getIndex().getName();

        _ss.str("");
        if (lnode.getIterationCountNode() != nullptr) {
            _ss << "for " << jj << " ∈ [0, " << lnode.getIterationCountNode()->getIndex().getName() << "-1]";
        } else {
            _ss << "for " << jj << " ∈ [0, " << (lnode.getIterationCount() - 1) << "]";
        }
        std::string name = printNodeDeclaration(node, _ss, "parallelogram");

        /**
         * connections
         */
        if (lnode.getIterationCountNode() != nullptr) {
            // is label ready necessary?
            printEdge(*lnode.getIterationCountNode(), name, "label=\"" + (*lnode.getIterationCountNode()->getIndex().getName()) + "\"");
            _code << _endline;
        }

        printEdge(lnode.getIndex(), name, "label=\"index " + jj + "\"");
        _code << _endline;

        return name;
    }

    virtual std::string printLoopEnd(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopEnd, "Invalid node type")

        std::string name = printNodeDeclaration(node);

        printEdges(name, node, "color=grey");

        _currentLoops.pop_back();

        return name;
    }

    virtual std::string printLoopIndexedDep(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for loop indexed dependent operation")

        std::string name = printNodeDeclaration(node);

        // LoopIndexedDep
        printEdges(name, node);

        return name;
    }

    virtual std::string printLoopIndexedIndep(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedIndep, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getInfo().size() == 1, "Invalid number of information elements for loop indexed independent operation")

        size_t pos = node.getInfo()[1];
        const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
        _ss << _nameGen->generateIndexedIndependent(node, getVariableID(node), *ip);

        std::string name = printNodeDeclaration(node, _ss);

        printEdges(name, node);

        return name;
    }

    virtual std::string printLoopIndexedTmp(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedTmp, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for loop indexed temporary operation")
        OperationNode<Base>* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")

        std::string name = printNodeDeclaration(node);

        printEdges(name, node);

        return name;
    }

    virtual std::string printTmpVar(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Tmp, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for temporary variable usage operation")
#ifndef NDEBUG
        OperationNode<Base>* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")
#endif
        // do nothing

        return makeNodeName(node);
    }

    virtual std::string printIndexAssign(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexAssign, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for an index assignment operation")

        auto& inode = static_cast<IndexAssignOperationNode<Base>&> (node);

        const IndexPattern& ip = inode.getIndexPattern();
        _ss.str("");
        _ss << (*inode.getIndex().getName()) << " = ";
        indexPattern2String(_ss, ip, inode.getIndexPatternIndexes());

        std::string name = printNodeDeclaration(node, _ss);

        /**
         * Connections
         */
        for (const auto* idx: inode.getIndexPatternIndexes()) {
            printEdge(*idx, name);
            _code << "  ";
        }
        _code << _endline;

        return name;
    }

    virtual std::string printIndexCondExprOp(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexCondExpr, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for an index condition expression operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an index condition expression operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation()->getOperationType() == CGOpCode::Index, "Invalid argument for an index condition expression operation")

        const std::vector<size_t>& info = node.getInfo();

        auto& iterationIndexOp = static_cast<IndexOperationNode<Base>&> (*node.getArguments()[0].getOperation());
        const std::string& index = *iterationIndexOp.getIndex().getName();

        _ss.str("");
        printIndexCondExpr(_ss, info, index);

        std::string name = printNodeDeclaration(node, _ss);

        return name;
    }

    virtual std::string printStartIf(OperationNode<Base>& node) {
        /**
         * the first argument is the condition, following arguments are
         * just extra dependencies that must be defined outside the if
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::StartIf, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'if start' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'if start' operation")

        //printIndexCondExprOp(*node.getArguments()[0].getOperation());
        std::string name = printNodeDeclaration(node, "", "diamond");

        printEdges(name, node, std::vector<std::string>{},
                   std::vector<std::string>{"label=\"condition\""});

        return name;
    }

    virtual std::string printElseIf(OperationNode<Base>& node) {
        /**
         * the first argument is the condition, the second argument is the 
         * if start node, the following arguments are assignments in the
         * previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::ElseIf, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 2, "Invalid number of arguments for an 'else if' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'else if' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an 'else if' operation")

        std::string name = printNodeDeclaration(node, "", "diamond");

        printEdges(name, node, std::vector<std::string>{},
                   std::vector<std::string>{"label=\"false\"", "label=\"condition\""});

        return name;
    }

    virtual std::string printElse(OperationNode<Base>& node) {
        /**
         * the first argument is the  if start node, the following arguments
         * are assignments in the previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Else, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'else' operation")

        std::string name = printNodeDeclaration(node, "", "diamond");

        printEdges(name, node, std::vector<std::string>{},
                   std::vector<std::string>{"label=\"false\""});

        return name;
    }

    virtual std::string printEndIf(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::EndIf, "Invalid node type for an 'end if' operation")

        std::string name = printNodeDeclaration(node, "", "diamond");

        printEdges(name, node);

        return name;
    }

    virtual std::string printCondResult(OperationNode<Base>& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::CondResult, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for an assignment inside an if/else operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation")

        print(node.getArguments()[0]); // condition start (e.g if or else if)
        print(node.getArguments()[1]); // temporary result

        std::string name = printNodeDeclaration(node, "", "diamond");

        printEdges(name, node);

        return name;
    }

    inline bool isDependent(const OperationNode<Base>& arg) const {
        if (arg.getOperationType() == CGOpCode::LoopIndexedDep) {
            return true;
        }
        size_t id = getVariableID(arg);
        return id > _independentSize && id < _minTemporaryVarID;
    }

    virtual void getComparison(std::ostream& os, enum CGOpCode op) const {
        switch (op) {
            case CGOpCode::ComLt:
                os << "<";
                return;

            case CGOpCode::ComLe:
                os << "≤";
                return;

            case CGOpCode::ComEq:
                os << "==";
                return;

            case CGOpCode::ComGe:
                os << "≥";
                return;

            case CGOpCode::ComGt:
                os << ">";
                return;

            case CGOpCode::ComNe:
                os << "≠";
                return;

            default:
                CPPAD_ASSERT_UNKNOWN(0)
                break;
        }
        throw CGException("Invalid comparison operator code"); // should never get here
    }

    static bool isFunction(enum CGOpCode op) {
        return isUnaryFunction(op) || op == CGOpCode::Pow;
    }

    static bool isUnaryFunction(enum CGOpCode op) {
        switch (op) {
            case CGOpCode::Abs:
            case CGOpCode::Acos:
            case CGOpCode::Asin:
            case CGOpCode::Atan:
            case CGOpCode::Cosh:
            case CGOpCode::Cos:
            case CGOpCode::Exp:
            case CGOpCode::Log:
            case CGOpCode::Sign:
            case CGOpCode::Sinh:
            case CGOpCode::Sin:
            case CGOpCode::Sqrt:
            case CGOpCode::Tanh:
            case CGOpCode::Tan:
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
            case CGOpCode::Erfc:
            case CGOpCode::Asinh:
            case CGOpCode::Acosh:
            case CGOpCode::Atanh:
            case CGOpCode::Expm1:
            case CGOpCode::Log1p:
#endif
                return true;
            default:
                return false;
        }
    }

    static bool isCondAssign(enum CGOpCode op) {
        switch (op) {
            case CGOpCode::ComLt:
            case CGOpCode::ComLe:
            case CGOpCode::ComEq:
            case CGOpCode::ComGe:
            case CGOpCode::ComGt:
            case CGOpCode::ComNe:
                return true;
            default:
                return false;
        }
    }
};

template<class Base>
const std::string LanguageDot<Base>::_C_STATIC_INDEX_ARRAY = "index"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageDot<Base>::_C_SPARSE_INDEX_ARRAY = "idx"; // NOLINT(cert-err58-cpp)

} // END cg namespace
} // END CppAD namespace

#endif
