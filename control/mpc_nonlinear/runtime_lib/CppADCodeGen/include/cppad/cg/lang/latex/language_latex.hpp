#ifndef CPPAD_CG_LANGUAGE_LATEX_INCLUDED
#define CPPAD_CG_LANGUAGE_LATEX_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2014 Ciengis
 *    Copyright (C) 2018 Joao Leal
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
 * Generates code for the Latex language.
 * It creates a generic implementation using custom latex environments which
 * must be implemented by the user.
 *
 * @author Joao Leal
 */
template<class Base>
class LanguageLatex : public Language<Base> {
public:
    using Node = OperationNode<Base>;
    using Arg = Argument<Base>;
protected:
    static const std::string _C_STATIC_INDEX_ARRAY;
    static const std::string _C_SPARSE_INDEX_ARRAY;
    static const std::string _COMP_OP_LT;
    static const std::string _COMP_OP_LE;
    static const std::string _COMP_OP_EQ;
    static const std::string _COMP_OP_GE;
    static const std::string _COMP_OP_GT;
    static const std::string _COMP_OP_NE;
    static const std::string _ATOMIC_TX;
    static const std::string _ATOMIC_TY;
    static const std::string _ATOMIC_PX;
    static const std::string _ATOMIC_PY;
protected:
    // information from the code handler (not owned)
    LanguageGenerationData<Base>* _info;
    // current indentation
    size_t _indentationLevel;
    // text before a variable name
    std::string _startVar;
    // text after a variable name
    std::string _endVar;
    // text before a dependent variable name
    std::string _startDepVar;
    // text after a dependent variable name
    std::string _endDepVar;
    // text before an independent variable name
    std::string _startIndepVar;
    // text after an independent variable name
    std::string _endIndepVar;
    // text before an individual equation
    std::string _startEq;
    // text after an individual equation
    std::string _endEq;
    // text before a line in the algorithm
    std::string _startAlgLine;
    // text after a line in the algorithm
    std::string _endAlgLine;
    // text before an equation block with multiple lines with the same indentation
    std::string _startEqBlock;
    // text after an equation block with multiple lines with the same indentation
    std::string _endEqBlock;
    std::string _algFileStart;
    std::string _algFileEnd;
    std::string _forStart;
    std::string _forEnd;
    std::string _conditionStart;
    std::string _conditionEnd;
    std::string _ifStart;
    std::string _ifEnd;
    std::string _elseIfStart;
    std::string _elseIfEnd;
    std::string _elseStart;
    std::string _elseEnd;
    std::string _assignStr;
    // markup for multiplications
    std::string _multOpStr;
    // markup for multiplications with parameters
    std::string _multValOpStr;
    // new line characters
    std::string _endline;
    // output stream for the generated source code
    std::ostringstream _code;
    // creates the variable names
    VariableNameGenerator<Base>* _nameGen;
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
    // the temporary variables that may require a declaration
    std::map<size_t, Node*> _temporary;
    // whether or not to ignore assignment of constant zero values to dependent variables
    bool _ignoreZeroDepAssign;
    // the name of the file to be created without the extension
    std::string _filename;
    // the maximum number of assignment (~lines) per local file
    size_t _maxAssignmentsPerFile;
    //
    std::map<std::string, std::string>* _sources;
    // the values in the temporary array
    std::vector<const Arg*> _tmpArrayValues;
    // the values in the temporary sparse array
    std::vector<const Arg*> _tmpSparseArrayValues;
    //
    std::vector<const LoopStartOperationNode<Base>*> _currentLoops;
    // the maximum precision used to print values
    size_t _parameterPrecision;
    // whether or not we are in an equation/align block
    bool _inEquationEnv;
    // whether or not to always enclose the base of a power within parenthesis
    bool _powBaseEnclose;
private:
    std::string auxArrayName_;

public:

    /**
     * Creates a Latex language source code generator
     */
    LanguageLatex() :
        _info(nullptr),
        _indentationLevel(0),
        _startVar("\\begin{CGVar}"),
        _endVar("\\end{CGVar}"),
        _startDepVar("\\begin{CGDepVar}"),
        _endDepVar("\\end{CGDepVar}"),
        _startIndepVar("\\begin{CGIndVar}"),
        _endIndepVar("\\end{CGIndVar}"),
        _startEq("\\begin{CGEq}"),
        _endEq("\\end{CGEq}"),
        _startAlgLine("\\begin{CGLine}"),
        _endAlgLine("\\end{CGLine}"),
        _startEqBlock("\\begin{CGEqBlock}"),
        _endEqBlock("\\end{CGEqBlock}"),
        _algFileStart("\\begin{CGAlgFile}"),
        _algFileEnd("\\end{CGAlgFile}"),
        _forStart("\\begin{CGFor}"),
        _forEnd("\\end{CGFor}"),
        _conditionStart("\\begin{CGCond}"),
        _conditionEnd("\\end{CGCond}"),
        _ifStart("\\begin{CGIf}"),
        _ifEnd("\\end{CGIf}"),
        _elseIfStart("\\begin{CGElseIf}"),
        _elseIfEnd("\\end{CGElseIf}"),
        _elseStart("\\begin{CGElse}"),
        _elseEnd("\\end{CGElse}"),
        _assignStr(" = "),
        _multOpStr(" "),
        _multValOpStr("\\times"),
        _endline("\n"),
        _nameGen(nullptr),
        _independentSize(0), // not really required (but it avoids warnings)
        _minTemporaryVarID(0), // not really required (but it avoids warnings)
        _dependent(nullptr),
        _ignoreZeroDepAssign(false),
        _filename("algorithm"),
        _maxAssignmentsPerFile(0),
        _sources(nullptr),
        _parameterPrecision(std::numeric_limits<Base>::digits10),
        _inEquationEnv(false),
        _powBaseEnclose(false) {
    }

    inline const std::string& getAssignString() const {
        return _assignStr;
    }

    inline void setAssignString(const std::string& assign) {
        _assignStr = assign;
    }

    inline bool isIgnoreZeroDepAssign() const {
        return _ignoreZeroDepAssign;
    }

    inline void setIgnoreZeroDepAssign(bool ignore) {
        _ignoreZeroDepAssign = ignore;
    }

    virtual void setFilename(const std::string& name) {
        _filename = name;
    }

    /**
     * Defines the Latex environment for each variable.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setVariableEnvironment(const std::string& begin,
                                        const std::string& end) {
        _startVar = begin;
        _endVar = end;
    }

    /**
     * Provides the string used to create the environment for each variable.
     */
    virtual const std::string& getVariableEnvironmentStart() const {
        return _startVar;
    }

    /**
     * Provides the string used to terminate the environment for each variable.
     */
    virtual const std::string& getVariableEnvironmentEnd() const {
        return _endVar;
    }

    /**
     * Defines the Latex environment for each dependent variable.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setDependentVarEnvironment(const std::string& begin,
                                            const std::string& end) {
        _startDepVar = begin;
        _endDepVar = end;
    }

    /**
     * Provides the string used to create the environment for each dependent variable.
     */
    virtual const std::string& getDependentVarEnvironmentStart() const {
        return _startDepVar;
    }

    /**
     * Provides the string used to terminate the environment for each dependent variable.
     */
    virtual const std::string& getDependentVarEnvironmentEnd() const {
        return _endDepVar;
    }

    /**
     * Defines the Latex environment for each independent variable.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setIndependentVarEnvironment(const std::string& begin,
                                              const std::string& end) {
        _startIndepVar = begin;
        _endIndepVar = end;
    }

    /**
     * Provides the string used to create the environment for each independent variable.
     */
    virtual const std::string& getIndependentVarEnvironmentStart() const {
        return _startIndepVar;
    }

    /**
     * Provides the string used to terminate the environment for each independent variable.
     */
    virtual const std::string& getIndependentVarEnvironmentEnd() const {
        return _endIndepVar;
    }

    /**
     * Defines the Latex environment for each equation.
     *
     * @param begin a string creating the environment (e.g. "$", "\\begin{algomathdisplay}", "\\[")
     * @param end a string terminating the environment (e.g. "$\\;", "\\end{algomathdisplay}\\;", "\\]\\;")
     */
    virtual void setEquationEnvironment(const std::string& begin,
                                        const std::string& end) {
        _startEq = begin;
        _endEq = end;
    }

    /**
     * Provides the string used to create the environment for each equation.
     */
    virtual const std::string& getEquationEnvironmentStart() const {
        return _startEq;
    }

    /**
     * Provides the string used to terminate the environment for each equation.
     */
    virtual const std::string& getEquationEnvironmentEnd() const {
        return _endEq;
    }

    /**
     * Defines the Latex environment for each algorithm line.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setAlgorithmLineEnvironment(const std::string& begin,
                                             const std::string& end) {
        _startAlgLine = begin;
        _endAlgLine = end;
    }

    /**
     * Provides the string used to create the environment for each line.
     */
    virtual const std::string& getAlgorithmLineEnvironmentStart() const {
        return _startAlgLine;
    }

    /**
     * Provides the string used to terminate the environment for each line.
     */
    virtual const std::string& getAlgorithmLineEnvironmentEnd() const {
        return _endAlgLine;
    }

    /**
     * Defines the Latex environment for each equation block which can contain
     * multiple equation lines with the same indentation.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setEquationBlockEnvironment(const std::string& begin,
                                             const std::string& end) {
        _startEqBlock = begin;
        _endEqBlock = end;
    }

    /**
     * Provides the string used to create the environment for each equation
     * block which can contain multiple equation lines with the same indentation.
     */
    virtual const std::string& getEquationBlockEnvironmentStart() const {
        return _startEqBlock;
    }

    /**
     * Provides the string used to terminate the environment for each equation
     * block which can contain multiple equation lines with the same indentation.
     */
    virtual const std::string& getEquationBlockEnvironmentEnd() const {
        return _endEqBlock;
    }

    /**
     * Defines the Latex environment for each algorithm file.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setAgorithmFileEnvironment(const std::string& begin,
                                            const std::string& end) {
        _algFileStart = begin;
        _algFileEnd = end;
    }

    /**
     * Provides the string used to create the environment for each algorithm file.
     */
    virtual const std::string& getAgorithmFileEnvironmentStart() const {
        return _algFileStart;
    }

    /**
     * Provides the string used to terminate the environment for each algorithm file.
     */
    virtual const std::string& getAgorithmFileEnvironmentEnd() const {
        return _algFileEnd;
    }

    /**
     * Defines the Latex environment for each for loop.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setForEnvironment(const std::string& begin,
                                   const std::string& end) {
        _forStart = begin;
        _forEnd = end;
    }

    /**
     * Provides the string used to create the environment for each for loop.
     */
    virtual const std::string& getForEnvironmentStart() const {
        return _forStart;
    }

    /**
     * Provides the string used to terminate the environment for each for loop.
     */
    virtual const std::string& getForEnvironmentEnd() const {
        return _forEnd;
    }

    /**
     * Defines the Latex environment for each condition.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setConditionEnvironment(const std::string& begin,
                                         const std::string& end) {
        _conditionStart = begin;
        _conditionEnd = end;
    }

    /**
     * Provides the string used to create the environment for each condition.
     */
    virtual const std::string& getConditionEnvironmentStart() const {
        return _conditionStart;
    }

    /**
     * Provides the string used to terminate the environment for each condition.
     */
    virtual const std::string& getConditionEnvironmentEnd() const {
        return _conditionEnd;
    }

    /**
     * Defines the Latex environment for each If.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setIfEnvironment(const std::string& begin,
                                  const std::string& end) {
        _ifStart = begin;
        _ifEnd = end;
    }

    /**
     * Provides the string used to create the environment for each If.
     */
    virtual const std::string& getIfEnvironmentStart() const {
        return _ifStart;
    }

    /**
     * Provides the string used to terminate the environment for each If.
     */
    virtual const std::string& getIfEnvironmentEnd() const {
        return _ifEnd;
    }

    /**
     * Defines the Latex environment for each else if.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setElseIfEnvironment(const std::string& begin,
                                      const std::string& end) {
        _elseIfStart = begin;
        _elseIfEnd = end;
    }

    /**
     * Provides the string used to create the environment for each else if.
     */
    virtual const std::string& getElseIfEnvironmentStart() const {
        return _elseIfStart;
    }

    /**
     * Provides the string used to terminate the environment for each else if.
     */
    virtual const std::string& getElseIfEnvironmentEnd() const {
        return _elseIfEnd;
    }

    /**
     * Defines the Latex environment for each else.
     *
     * @param begin a string creating the environment
     * @param end a string terminating the environment
     */
    virtual void setElseEnvironment(const std::string& begin,
                                    const std::string& end) {
        _elseStart = begin;
        _elseEnd = end;
    }

    /**
     * Provides the string used to create the environment for each else.
     */
    virtual const std::string& getElseEnvironmentStart() const {
        return _elseStart;
    }

    /**
     * Provides the string used to terminate the environment for each else.
     */
    virtual const std::string& getElseEnvironmentEnd() const {
        return _elseEnd;
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
     * Defines whether or not to always enclose the base of a power within
     * parenthesis.
     * By default the base is only enclosed in parenthesis if it contains of a
     * mathematical expression.
     *
     * @param enclose true to always enclose the base in parenthesis
     */
    virtual void setAlwaysEnclosePowBase(bool enclose) {
        _powBaseEnclose = enclose;
    }

    /**
     * Whether or not to always enclose the base of a power within
     * parenthesis.
     *
     * @return true if the base is always enclosed within parenthesis
     */
    virtual bool isAlwaysEnclosePowBase() const {
        return _powBaseEnclose;
    }


    /**
     * Provides the operator used to define multiplications.
     * The default is an empty space.
     *
     * @note Multiplications of constant parameters use a different
     *       multiplication markup string
     */
    inline const std::string& getMultiplicationOperator() const {
        return _multOpStr;
    }

    /**
     * Defines the operator used for multiplications.
     * The default is an empty space.
     * Other common alternatives are "\cdot" and "\times".
     *
     * @note Multiplications of constant parameters use a different
     *       multiplication operator
     */
    inline void setMultiplicationOperator(const std::string& multOpStr) {
        _multOpStr = multOpStr;
    }

    /**
     * Provides the operator used for multiplication of constant parameters.
     * The default is "\times".
     */
    inline const std::string& getMultiplicationConstParOperator() const {
        return _multValOpStr;
    }

    /**
     * Defines the operator used for multiplication of constant parameters.
     * The default is "\times".
     * Another common alternative is "\cdot".
     * Please take into account that numbers too close together are difficult
     * to distinguish.
     */
    inline void setMultiplicationConstParOperator(const std::string& multValOpStr) {
        _multValOpStr = multValOpStr;
    }

    virtual void setMaxAssignmentsPerFunction(size_t maxAssignmentsPerFunction,
                                              std::map<std::string, std::string>* sources) {
        _maxAssignmentsPerFile = maxAssignmentsPerFunction;
        _sources = sources;
    }

    inline virtual ~LanguageLatex() = default;

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
                out << " \\vee "; // or
            }
            size_t min = info[e];
            size_t max = info[e + 1];
            if (min == max) {
                out << index << " == " << min;
            } else if (min == 0) {
                out << index << " \\le " << max;
            } else if (max == (std::numeric_limits<size_t>::max)()) {
                out << min << " \\le " << index;
            } else {
                if (infoSize != 2)
                    out << "(";

                if (max - min == 1)
                    out << min << " == " << index << " \\vee " << index << " == " << max;
                else
                    out << min << " \\le " << index << " \\wedge" << index << " \\le " << max;

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
                                                   const std::string& identation,
                                                   const std::set<RandomIndexPattern*>& randomPatterns);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const Node& index);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::vector<const Node*>& indexes);

    static inline std::string linearIndexPattern2String(const LinearIndexPattern& lip,
                                                        const Node& index);

    /***************************************************************************
     *                              protected
     **************************************************************************/
protected:

    void generateSourceCode(std::ostream& out,
                            std::unique_ptr<LanguageGenerationData<Base> > info) override {

        const bool multiFile = _maxAssignmentsPerFile > 0 && _sources != nullptr;

        // clean up
        _code.str("");
        _ss.str("");
        _indentationLevel = 0;
        _temporary.clear();
        _inEquationEnv = false;
        auxArrayName_ = "";
        _currentLoops.clear();
        _dependentIDs.clear();


        // save some info
        _info = info.get();
        _independentSize = info->independent.size();
        _dependent = &info->dependent;
        _nameGen = &info->nameGen;
        _minTemporaryVarID = info->minTemporaryVarID;
        const ArrayView<CG<Base> >& dependent = info->dependent;
        const std::vector<Node*>& variableOrder = info->variableOrder;

        _tmpArrayValues.resize(_nameGen->getMaxTemporaryArrayVariableID());
        std::fill(_tmpArrayValues.begin(), _tmpArrayValues.end(), nullptr);
        _tmpSparseArrayValues.resize(_nameGen->getMaxTemporarySparseArrayVariableID());
        std::fill(_tmpSparseArrayValues.begin(), _tmpSparseArrayValues.end(), nullptr);

        /**
         * generate index array names (might be used for variable names)
         */
        generateNames4RandomIndexPatterns(info->indexRandomPatterns);

        /**
         * generate variable names
         */
        //generate names for the independent variables
        for (size_t j = 0; j < _independentSize; j++) {
            Node& op = *info->independent[j];
            if (op.getName() == nullptr) {
                op.setName(_nameGen->generateIndependent(op, getVariableID(op)));
            }
        }

        // generate names for the dependent variables (must be after naming independents)
        for (size_t i = 0; i < dependent.size(); i++) {
            Node* node = dependent[i].getOperationNode();
            if (node != nullptr && node->getOperationType() != CGOpCode::LoopEnd && node->getName() == nullptr) {
                if (node->getOperationType() == CGOpCode::LoopIndexedDep) {
                    size_t pos = node->getInfo()[0];
                    const IndexPattern* ip = info->loopDependentIndexPatterns[pos];
                    node->setName(_nameGen->generateIndexedDependent(*node, getVariableID(*node), *ip));

                } else {
                    node->setName(_nameGen->generateDependent(i));
                }
            }
        }

        /**
         * function variable declaration
         */
        const std::vector<FuncArgument>& indArg = _nameGen->getIndependent();
        const std::vector<FuncArgument>& depArg = _nameGen->getDependent();
        const std::vector<FuncArgument>& tmpArg = _nameGen->getTemporary();
        CPPADCG_ASSERT_KNOWN(!indArg.empty() && !depArg.empty(),
                             "There must be at least one dependent and one independent argument")
        CPPADCG_ASSERT_KNOWN(tmpArg.size() == 3,
                             "There must be three temporary variables")

        auxArrayName_ = tmpArg[1].name + "p";

        /**
         * Determine the dependent variables that result from the same operations
         */
        // dependent variables indexes that are copies of other dependent variables
        std::set<size_t> dependentDuplicates;

        for (size_t i = 0; i < dependent.size(); i++) {
            Node* node = dependent[i].getOperationNode();
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

        // the names of local functions
        std::vector<std::string> inputLatexFiles;
        if (multiFile) {
            inputLatexFiles.reserve(variableOrder.size() / _maxAssignmentsPerFile);
        }

        /**
         * non-constant variables
         */
        if (variableOrder.size() > 0) {
            // generate names for temporary variables
            for (Node* node : variableOrder) {
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
            if (info->zeroDependents) {
                // zero initial values
                if (!depArg.empty())
                    checkEquationEnvStart();
                for (size_t i = 0; i < depArg.size(); i++) {
                    _code << _startAlgLine << _startEq;
                    const FuncArgument& a = depArg[i];
                    if (a.array) {
                        _code << a.name;
                    } else {
                        _code << _startDepVar << _nameGen->generateDependent(i) << _endDepVar;
                    }
                    _code << _assignStr;
                    printParameter(Base(0.0));
                    _code << _endEq << _endAlgLine << _endline;
                }
            }

            size_t assignCount = 0;
            for (Node* it : variableOrder) {
                // check if a new function should start
                if (assignCount >= _maxAssignmentsPerFile && multiFile && _currentLoops.empty()) {
                    assignCount = 0;
                    saveLocalFunction(inputLatexFiles, inputLatexFiles.empty() && info->zeroDependents);
                }

                Node& node = *it;

                // a dependent variable assigned by a loop does require any source code (its done inside the loop)
                if (node.getOperationType() == CGOpCode::DependentRefRhs) {
                    continue; // nothing to do (this operation is right hand side only)
                } else if (node.getOperationType() == CGOpCode::TmpDcl) { // temporary variable declaration does not need any source code here
                    continue; // nothing to do (bogus operation)
                }

                assignCount += printAssignment(node);
            }

            if (!inputLatexFiles.empty() && assignCount > 0) {
                assignCount = 0;
                saveLocalFunction(inputLatexFiles, false);
            }
        }

        if (!inputLatexFiles.empty()) {
            /**
             * Create the master latex file which inputs the other files
             */
            CPPADCG_ASSERT_KNOWN(tmpArg[0].array,
                                 "The temporary variables must be saved in an array in order to generate multiple functions")
            printAlgorithmFileStart(_code);
            for (auto & inputLatexFile : inputLatexFiles) {
                _code << "\\input{" << inputLatexFile << "}" << _endline;
            }
            printAlgorithmFileEnd(_code);
        }

        // dependent duplicates
        if (!dependentDuplicates.empty()) {
            _code << "% variable duplicates: " << dependentDuplicates.size() << _endline;

            checkEquationEnvStart();
            for (size_t index : dependentDuplicates) {
                const CG<Base>& dep = (*_dependent)[index];
                std::string varName = _nameGen->generateDependent(index);
                const std::string& origVarName = *dep.getOperationNode()->getName();

                _code << _startAlgLine << _startEq
                        << _startDepVar << varName << _endDepVar
                        << _assignStr
                        << _startDepVar << origVarName << _endDepVar;
                printAssignmentEnd();
            }
        }

        // constant dependent variables
        bool commentWritten = false;
        for (size_t i = 0; i < dependent.size(); i++) {
            if (dependent[i].isParameter()) {
                if (!_ignoreZeroDepAssign || !dependent[i].isIdenticalZero()) {
                    if (!commentWritten) {
                        _code << "% dependent variables without operations" << _endline;
                        commentWritten = true;
                    }
                    checkEquationEnvStart();

                    std::string varName = _nameGen->generateDependent(i);
                    _code << _startAlgLine << _startEq
                            << _startDepVar << varName << _endDepVar << _assignStr;
                    printParameter(dependent[i].getValue());
                    printAssignmentEnd();
                }
            } else if (dependent[i].getOperationNode()->getOperationType() == CGOpCode::Inv) {
                if (!commentWritten) {
                    _code << "% dependent variables without operations" << _endline;
                    commentWritten = true;
                }
                checkEquationEnvStart();

                std::string varName = _nameGen->generateDependent(i);
                const std::string& indepName = *dependent[i].getOperationNode()->getName();
                _code << _startAlgLine << _startEq
                        << _startDepVar << varName << _endDepVar
                        << _assignStr
                        << _startIndepVar << indepName << _endIndepVar;
                printAssignmentEnd(*dependent[i].getOperationNode());
            }
        }

        checkEquationEnvEnd();

        /**
         * encapsulate the code in a function
         */
        if (inputLatexFiles.empty()) {
            // a single source file
            printAlgorithmFileStart(_ss);
            _ss << _code.str();
            printAlgorithmFileEnd(_ss);

            out << _ss.str();

            if (_sources != nullptr) {
                (*_sources)[_filename + ".tex"] = _ss.str();
            }
        } else {
            // there are multiple source files (this last one is the master)
            (*_sources)[_filename + ".tex"] = _code.str();
        }

    }

    inline size_t getVariableID(const Node& node) const {
        return _info->varId[node];
    }

    inline virtual void printAlgorithmFileStart(std::ostream& out) {
        out << "% Latex source file for '" << _filename << "' (automatically generated by CppADCodeGen)" << _endline;
        out << _algFileStart << _endline;
    }

    inline virtual void printAlgorithmFileEnd(std::ostream& out) {
        out << _algFileEnd;
    }

    inline virtual void checkEquationEnvStart() {
        if (!_inEquationEnv) {
            _code << _startEqBlock << _endline;
            _inEquationEnv = true;
        }
    }

    inline virtual void checkEquationEnvEnd() {
        if (_inEquationEnv) {
            _code << _endEqBlock << _endline;
            _inEquationEnv = false;
        }
    }

    inline unsigned printAssignment(Node& node) {
        return printAssignment(node, node);
    }

    inline unsigned printAssignment(Node& nodeName,
                                   const Arg& nodeRhs) {
        if (nodeRhs.getOperation() != nullptr) {
            return printAssignment(nodeName, *nodeRhs.getOperation());
        } else {
            printAssignmentStart(nodeName);
            printParameter(*nodeRhs.getParameter());
            printAssignmentEnd(nodeName);
            return 1;
        }
    }

    inline unsigned printAssignment(Node& nodeName,
                                   Node& nodeRhs) {
        bool createsVar = directlyAssignsVariable(nodeRhs); // do we need to do the assignment here?
        if (!createsVar) {
            printAssignmentStart(nodeName);
        }
        unsigned lines = printExpressionNoVarCheck(nodeRhs);
        if (!createsVar) {
            printAssignmentEnd(nodeRhs);
        }

        if (nodeRhs.getOperationType() == CGOpCode::ArrayElement) {
            Node* array = nodeRhs.getArguments()[0].getOperation();
            size_t arrayId = getVariableID(*array);
            size_t pos = nodeRhs.getInfo()[0];
            if (array->getOperationType() == CGOpCode::ArrayCreation)
                _tmpArrayValues[arrayId - 1 + pos] = nullptr; // this could probably be removed!
            else
                _tmpSparseArrayValues[arrayId - 1 + pos] = nullptr; // this could probably be removed!
        }

        return lines;
    }

    inline virtual void printAssignmentStart(Node& op) {
        printAssignmentStart(op, createVariableName(op), isDependent(op));
    }

    inline virtual void printAssignmentStart(Node& node, const std::string& varName, bool isDep) {
        if (!isDep) {
            _temporary[getVariableID(node)] = &node;
        }

        checkEquationEnvStart();

        _code << _startAlgLine << _startEq;
        if (isDep)
            _code << _startDepVar << varName << _endDepVar;
        else
            _code << _startVar << varName << _endVar;

        CGOpCode op = node.getOperationType();
        if (op == CGOpCode::DependentMultiAssign || (op == CGOpCode::LoopIndexedDep && node.getInfo()[1] == 1)) {
            _code << " += ";
        } else {
            _code << _assignStr;
        }
    }

    inline virtual void printAssignmentEnd() {
        _code << _endEq << _endAlgLine << _endline;
    }

    inline virtual void printAssignmentEnd(Node& op) {
        printAssignmentEnd();
    }

    virtual void saveLocalFunction(std::vector<std::string>& localFuncNames,
                                   bool zeroDependentArray) {
        _ss << _filename << "__part_" << (localFuncNames.size() + 1);
        std::string funcName = _ss.str();
        _ss.str("");

        // loop indexes
        _nameGen->prepareCustomFunctionVariables(_ss);
        _ss << _code.str();
        _nameGen->finalizeCustomFunctionVariables(_ss);

        (*_sources)[funcName + ".tex"] = _ss.str();
        localFuncNames.push_back(funcName);

        _code.str("");
        _ss.str("");
    }

    bool createsNewVariable(const Node& var,
                            size_t totalUseCount,
                            size_t opCount) const override {
        CGOpCode op = var.getOperationType();
        if (totalUseCount > 1) {
            return op != CGOpCode::ArrayElement && op != CGOpCode::Index && op != CGOpCode::IndexDeclaration && op != CGOpCode::Tmp;
        } else {
            return ( op == CGOpCode::ArrayCreation ||
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

    virtual bool requiresVariableName(const Node& var) const {
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
    virtual bool directlyAssignsVariable(const Node& var) const {
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

    inline const std::string& createVariableName(Node& var) {
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
                CPPADCG_ASSERT_UNKNOWN(it != _dependentIDs.end())

                size_t index = it->second;
                var.setName(_nameGen->generateDependent(index));

            } else if (op == CGOpCode::LoopIndexedTmp || op == CGOpCode::Tmp) {
                CPPADCG_ASSERT_KNOWN(var.getArguments().size() >= 1, "Invalid number of arguments for loop indexed temporary operation")
                Node* tmpVar = var.getArguments()[0].getOperation();
                CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")
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

    virtual void printIndependentVariableName(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 0, "Invalid number of arguments for independent variable")

        _code << _startIndepVar << _nameGen->generateIndependent(op, getVariableID(op)) << _endIndepVar;
    }

    virtual unsigned print(const Arg& arg) {
        if (arg.getOperation() != nullptr) {
            // expression
            return printExpression(*arg.getOperation());
        } else {
            // parameter
            printParameter(*arg.getParameter());
            return 1;
        }
    }

    virtual unsigned printExpression(Node& node) {
        if (getVariableID(node) > 0) {
            const std::string& name = createVariableName(node); // use variable name

            CGOpCode op = node.getOperationType();
            if (getVariableID(node) >= _minTemporaryVarID || op == CGOpCode::ArrayCreation || op == CGOpCode::SparseArrayCreation || op == CGOpCode::LoopIndexedDep || op == CGOpCode::LoopIndexedIndep) {

                _code << _startVar << name << _endVar;

            } else if (getVariableID(node) <= _independentSize) {
                // independent variable
                _code << _startIndepVar << name << _endIndepVar;

            } else {
                // dependent variable
                _code << _startDepVar << name << _endDepVar;

            }

            return 1;
        } else {
            // print expression code
            return printExpressionNoVarCheck(node);
        }
    }

    virtual unsigned printExpressionNoVarCheck(Node& node) {
        CGOpCode op = node.getOperationType();
        switch (op) {
            case CGOpCode::ArrayCreation:
                printArrayCreationOp(node);
                break;
            case CGOpCode::SparseArrayCreation:
                printSparseArrayCreationOp(node);
                break;
            case CGOpCode::ArrayElement:
                printArrayElementOp(node);
                break;
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
                printUnaryFunction(node);
                break;
            case CGOpCode::AtomicForward: // atomicFunction.forward(q, p, vx, vy, tx, ty)
                printAtomicForwardOp(node);
                break;
            case CGOpCode::AtomicReverse: // atomicFunction.reverse(p, tx, ty, px, py)
                printAtomicReverseOp(node);
                break;
            case CGOpCode::Add:
                printOperationAdd(node);
                break;
            case CGOpCode::Alias:
                return printOperationAlias(node);

            case CGOpCode::ComLt:
            case CGOpCode::ComLe:
            case CGOpCode::ComEq:
            case CGOpCode::ComGe:
            case CGOpCode::ComGt:
            case CGOpCode::ComNe:
                printConditionalAssignment(node);
                break;
            case CGOpCode::Div:
                printOperationDiv(node);
                break;
            case CGOpCode::Inv:
                printIndependentVariableName(node);
                break;
            case CGOpCode::Mul:
                printOperationMul(node);
                break;
            case CGOpCode::Pow:
                printPowFunction(node);
                break;
            case CGOpCode::Pri:
                // do nothing
                break;
            case CGOpCode::Sub:
                printOperationMinus(node);
                break;

            case CGOpCode::UnMinus:
                printOperationUnaryMinus(node);
                break;

            case CGOpCode::DependentMultiAssign:
                return printDependentMultiAssign(node);

            case CGOpCode::Index:
                return 0; // nothing to do
            case CGOpCode::IndexAssign:
                printIndexAssign(node);
                break;
            case CGOpCode::IndexDeclaration:
                return 0; // already done

            case CGOpCode::LoopStart:
                printLoopStart(node);
                break;
            case CGOpCode::LoopIndexedIndep:
                printLoopIndexedIndep(node);
                break;
            case CGOpCode::LoopIndexedDep:
                printLoopIndexedDep(node);
                break;
            case CGOpCode::LoopIndexedTmp:
                printLoopIndexedTmp(node);
                break;
            case CGOpCode::TmpDcl:
                // nothing to do
                return 0;
            case CGOpCode::Tmp:
                printTmpVar(node);
                break;
            case CGOpCode::LoopEnd:
                printLoopEnd(node);
                break;
            case CGOpCode::IndexCondExpr:
                printIndexCondExprOp(node);
                break;
            case CGOpCode::StartIf:
                printStartIf(node);
                break;
            case CGOpCode::ElseIf:
                printElseIf(node);
                break;
            case CGOpCode::Else:
                printElse(node);
                break;
            case CGOpCode::EndIf:
                printEndIf(node);
                break;
            case CGOpCode::CondResult:
                printCondResult(node);
                break;
            case CGOpCode::UserCustom:
                printUserCustom(node);
                break;
            default:
                throw CGException("Unknown operation code '", op, "'.");
        }
        return 1;
    }

    virtual unsigned printAssignOp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for assign operation")

        return print(node.getArguments()[0]);
    }

    virtual void printUnaryFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary function")

        switch (op.getOperationType()) {
            case CGOpCode::Abs:
                _code << "\\abs{";
                print(op.getArguments()[0]);
                _code << "}";
                return;
            case CGOpCode::Acos:
                _code << "\\arccos";
                break;
            case CGOpCode::Asin:
                _code << "\\arcsin";
                break;
            case CGOpCode::Atan:
                _code << "\\arctan";
                break;
            case CGOpCode::Cosh:
                _code << "\\cosh";
                break;
            case CGOpCode::Cos:
                _code << "\\cos";
                break;
            case CGOpCode::Exp:
                _code << "\\exp"; ///////////////////////////////////////// consider using superscript
                break;
            case CGOpCode::Log:
                _code << "\\ln";
                break;
            case CGOpCode::Sinh:
                _code << "\\sinh";
                break;
            case CGOpCode::Sign:
                _code << "\\operatorname{sgn}";
                break;
            case CGOpCode::Sin:
                _code << "\\sin";
                break;
            case CGOpCode::Sqrt:
                _code << "\\sqrt{";
                print(op.getArguments()[0]);
                _code << "}";
                return;
            case CGOpCode::Tanh:
                _code << "\\tanh";
                break;
            case CGOpCode::Tan:
                _code << "\\tan";
                break;
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
                _code << "\\operatorname{erf}";
                break;
            case CGOpCode::Erfc:
                _code << "\\operatorname{erfc}";
                break;
            case CGOpCode::Asinh:
                _code << "\\operatorname{arcsinh}";
                break;
            case CGOpCode::Acosh:
                _code << "\\operatorname{arccosh}";
                break;
            case CGOpCode::Atanh:
                _code << "\\operatorname{arctanh}";
                break;
            case CGOpCode::Expm1:
                _code << "\\operatorname{expm1}";
                break;
            case CGOpCode::Log1p:
                _code << "\\operatorname{log1p}";
                break;
#endif
            default:
                throw CGException("Unknown function name for operation code '", op.getOperationType(), "'.");
        }

        _code << "\\mathopen{}\\left(";
        print(op.getArguments()[0]);
        _code << "\\right)\\mathclose{}";
    }

    virtual void printPowFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for pow() function")

        auto encloseInParentheses = [this](const Node* node) {
            while (node != nullptr) {
                if (getVariableID(*node) != 0)
                    return false;
                if (node->getOperationType() == CGOpCode::Alias)
                    node = node->getArguments()[0].getOperation();
                else
                    break;
            }
            return node != nullptr &&
                   getVariableID(*node) == 0 &&
                   !isFunction(node->getOperationType());
        };

        bool encloseBase = _powBaseEnclose || encloseInParentheses(op.getArguments()[0].getOperation());
        bool encloseExpo = encloseInParentheses(op.getArguments()[1].getOperation());

        _code << "{";
        if (encloseBase)
            _code << "\\left(";
        print(op.getArguments()[0]);
        if (encloseBase)
            _code << "\\right)";
        _code << "}^{";
        if (encloseExpo)
            _code << "\\left(";
        print(op.getArguments()[1]);
        if (encloseExpo)
            _code << "\\right)";
        _code << "}";
    }

    virtual unsigned printOperationAlias(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for alias")
        return print(op.getArguments()[0]);
    }

    virtual void printOperationAdd(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for addition")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        if(right.getParameter() == nullptr || (*right.getParameter() >= 0)) {
            print(left);
            _code << " + ";
            print(right);
        } else {
            // right has a negative parameter so we would get v0 + -v1
            print(left);
            _code << " - ";
            printParameter(-*right.getParameter()); // make it positive
        }
    }

    virtual void printOperationMinus(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for subtraction")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        if(right.getParameter() == nullptr || (*right.getParameter() >= 0)) {
            bool encloseRight = encloseInParenthesesMul(right);

            print(left);
            _code << " - ";
            if (encloseRight) {
                _code << "\\left(";
            }
            print(right);
            if (encloseRight) {
                _code << "\\right)";
            }
        } else {
            // right has a negative parameter so we would get v0 - -v1
            print(left);
            _code << " + ";
            printParameter(-*right.getParameter()); // make it positive
        }
    }

    virtual void printOperationDiv(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for division")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];


        _code << "\\frac{";
        print(left);
        _code << "}{";
        print(right);
        _code << "}";

    }

    inline bool encloseInParenthesesMul(const Arg& arg) const {
        if (arg.getParameter() != nullptr) {
            return ((*arg.getParameter()) < 0);
        } else {
            return encloseInParenthesesMul(arg.getOperation());
        }
    }

    inline bool encloseInParenthesesMul(const Node* node) const {
        while (node != nullptr) {
            if (getVariableID(*node) != 0) {
                return false;
            } else if (node->getOperationType() == CGOpCode::Alias) {
                node = node->getArguments()[0].getOperation();
            } else {
                break;
            }
        }
        return node != nullptr &&
                getVariableID(*node) == 0 &&
                node->getOperationType() != CGOpCode::Div &&
                node->getOperationType() != CGOpCode::Mul &&
                !isFunction(node->getOperationType());
    }

    virtual void printOperationMul(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for multiplication")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        bool encloseLeft = encloseInParenthesesMul(left);
        bool encloseRight = encloseInParenthesesMul(right);

        auto isNumber = [this](const Node* node, int pos) -> bool {
            while (node != nullptr) {
                if(getVariableID(*node) != 0) {
                    return false;
                }
                CGOpCode op = node->getOperationType();
                if (op == CGOpCode::Alias) {
                    node = node->getArguments()[0].getOperation();
                    continue;
                } else if (op == CGOpCode::Mul) {
                    node = node->getArguments()[pos].getOperation();
                } else if (pos == 0 && op == CGOpCode::Pow) {
                    node = node->getArguments()[0].getOperation();
                } else {
                    return false;
                }
            }
            return true; // a constant number
        };

        if (encloseLeft) {
            _code << "\\left(";
        }
        print(left);
        if (encloseLeft) {
            _code << "\\right)";
        }

        if (isNumber(left.getOperation(), 1) && isNumber(right.getOperation(), 0))
            _code << _multValOpStr; // numbers too close together are difficult to distinguish
        else
            _code << _multOpStr; // e.g. invisible times

        if (encloseRight) {
            _code << "\\left(";
        }
        print(right);
        if (encloseRight) {
            _code << "\\right)";
        }
    }

    virtual void printOperationUnaryMinus(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary minus")

        const Arg& arg = op.getArguments()[0];

        bool enclose = encloseInParenthesesMul(arg);

        _code << "-";
        if (enclose) {
            _code << "\\left(";
        }
        print(arg);
        if (enclose) {
            _code << "\\right)";
        }
    }

    virtual void printConditionalAssignment(Node& node) {
        CPPADCG_ASSERT_UNKNOWN(getVariableID(node) > 0)

        const std::vector<Arg>& args = node.getArguments();
        const Arg &left = args[0];
        const Arg &right = args[1];
        const Arg &trueCase = args[2];
        const Arg &falseCase = args[3];

        bool isDep = isDependent(node);
        const std::string& varName = createVariableName(node);

        if ((trueCase.getParameter() != nullptr && falseCase.getParameter() != nullptr && *trueCase.getParameter() == *falseCase.getParameter()) ||
                (trueCase.getOperation() != nullptr && falseCase.getOperation() != nullptr && trueCase.getOperation() == falseCase.getOperation())) {
            // true and false cases are the same
            printAssignmentStart(node, varName, isDep);
            print(trueCase);
            printAssignmentEnd(node);
        } else {
            checkEquationEnvEnd();

            _code << _ifStart;
            _code << _conditionStart;
            print(left);
            _code << " " << getComparison(node.getOperationType()) << " ";
            print(right);
            _code << _conditionEnd;
            _code << _endline;
            //checkEquationEnvStart(); // no need
            printAssignmentStart(node, varName, isDep);
            print(trueCase);
            printAssignmentEnd(node);
            checkEquationEnvEnd();
            _code << _ifEnd << _endline;
            _code << _elseStart << _endline; // else
            //checkEquationEnvStart(); // no need
            printAssignmentStart(node, varName, isDep);
            print(falseCase);
            printAssignmentEnd(node);
            checkEquationEnvEnd();
            _code << _elseEnd << _endline; // end if
        }
    }

    inline bool isSameArgument(const Arg& newArg,
                               const Arg* oldArg) {
        if (oldArg != nullptr) {
            if (oldArg->getParameter() != nullptr) {
                if (newArg.getParameter() != nullptr) {
                    return (*newArg.getParameter() == *oldArg->getParameter());
                }
            } else {
                return (newArg.getOperation() == oldArg->getOperation());
            }
        }
        return false;
    }

    virtual void printArrayCreationOp(Node& op);

    virtual void printSparseArrayCreationOp(Node& op);

    inline void printArrayStructInit(const std::string& dataArrayName,
                                     size_t pos,
                                     const std::vector<Node*>& arrays,
                                     size_t k);

    inline void printArrayStructInit(const std::string& dataArrayName,
                                     Node& array);

    inline void markArrayChanged(Node& ty);

    inline size_t printArrayCreationUsingLoop(size_t startPos,
                                              Node& array,
                                              size_t startj,
                                              std::vector<const Arg*>& tmpArrayValues);

    inline std::string getTempArrayName(const Node& op);

    virtual void printArrayElementOp(Node& op);

    virtual void printAtomicForwardOp(Node& atomicFor) {
        CPPADCG_ASSERT_KNOWN(atomicFor.getInfo().size() == 3, "Invalid number of information elements for atomic forward operation")
        int q = atomicFor.getInfo()[1];
        int p = atomicFor.getInfo()[2];
        size_t p1 = p + 1;
        const std::vector<Arg>& opArgs = atomicFor.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 2, "Invalid number of arguments for atomic forward operation")

        size_t id = atomicFor.getInfo()[0];
        std::vector<Node*> tx(p1), ty(p1);
        for (size_t k = 0; k < p1; k++) {
            tx[k] = opArgs[0 * p1 + k].getOperation();
            ty[k] = opArgs[1 * p1 + k].getOperation();
        }

        CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(ty[p]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")

        // tx
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_TX, k, tx, k);
        }
        // ty
        printArrayStructInit(_ATOMIC_TY, *ty[p]);
        _ss.str("");

        _code << _startAlgLine << _startEq
                << _info->atomicFunctionId2Name.at(id) << ".forward("
                << q << ", " << p << ", "
                << _ATOMIC_TX << ", &" << _ATOMIC_TY << ")"
                << _endEq << _endAlgLine << _endline;

        /**
         * the values of ty are now changed
         */
        markArrayChanged(*ty[p]);
    }

    virtual void printAtomicReverseOp(Node& atomicRev) {
        CPPADCG_ASSERT_KNOWN(atomicRev.getInfo().size() == 2, "Invalid number of information elements for atomic reverse operation")
        int p = atomicRev.getInfo()[1];
        size_t p1 = p + 1;
        const std::vector<Arg>& opArgs = atomicRev.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 4, "Invalid number of arguments for atomic reverse operation")

        size_t id = atomicRev.getInfo()[0];
        std::vector<Node*> tx(p1), px(p1), py(p1);
        for (size_t k = 0; k < p1; k++) {
            tx[k] = opArgs[0 * p1 + k].getOperation();
            px[k] = opArgs[2 * p1 + k].getOperation();
            py[k] = opArgs[3 * p1 + k].getOperation();
        }

        CPPADCG_ASSERT_KNOWN(tx[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(p == 0 || tx[1]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type")

        CPPADCG_ASSERT_KNOWN(px[0]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")

        CPPADCG_ASSERT_KNOWN(py[0]->getOperationType() == CGOpCode::SparseArrayCreation, "Invalid array type")
        CPPADCG_ASSERT_KNOWN(p == 0 || py[1]->getOperationType() == CGOpCode::ArrayCreation, "Invalid array type")

        // tx
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_TX, k, tx, k);
        }
        // py
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_PY, k, py, k);
        }
        // px
        printArrayStructInit(_ATOMIC_PX, *px[0]);
        _ss.str("");

        _code << _startAlgLine << _startEq
                << _info->atomicFunctionId2Name.at(id) << ".reverse("
                << p << ", "
                << _ATOMIC_TX << ", &" << _ATOMIC_PX << ", " << _ATOMIC_PY << ")"
                << _endEq << _endAlgLine << _endline;

        /**
         * the values of px are now changed
         */
        markArrayChanged(*px[0]);
    }

    virtual unsigned printDependentMultiAssign(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::DependentMultiAssign, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments")

        const std::vector<Arg>& args = node.getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            bool useArg = false;
            const Arg& arg = args[a];
            if (arg.getParameter() != nullptr) {
                useArg = true;
            } else {
                CGOpCode op = arg.getOperation()->getOperationType();
                useArg = op != CGOpCode::DependentRefRhs && op != CGOpCode::LoopEnd && op != CGOpCode::EndIf;
            }

            if (useArg) {
                printAssignment(node, arg); // ignore other arguments!
                return 1;
            }
        }
        return 0;
    }

    virtual void printLoopStart(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopStart, "Invalid node type")

        auto& lnode = static_cast<LoopStartOperationNode<Base>&> (node);
        _currentLoops.push_back(&lnode);

        const std::string& jj = *lnode.getIndex().getName();
        std::string lastIt;
        if (lnode.getIterationCountNode() != nullptr) {
            lastIt = *lnode.getIterationCountNode()->getIndex().getName() + " - 1";
        } else {
            lastIt = std::to_string(lnode.getIterationCount() - 1);
        }

        checkEquationEnvEnd();

        _code << _forStart << "{$" << jj << "\\in \\left[0, " << lastIt << "\\right]$}" << _endline;
        _indentationLevel++;
    }

    virtual void printLoopEnd(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopEnd, "Invalid node type")

        checkEquationEnvEnd();

        _indentationLevel--;

        _code << _forEnd << _endline;

        _currentLoops.pop_back();
    }

    virtual void printLoopIndexedDep(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for loop indexed dependent operation")

        // LoopIndexedDep
        print(node.getArguments()[0]);
    }

    virtual void printLoopIndexedIndep(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedIndep, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getInfo().size() == 1, "Invalid number of information elements for loop indexed independent operation")

        // CGLoopIndexedIndepOp
        size_t pos = node.getInfo()[1];
        const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
        _code << _nameGen->generateIndexedIndependent(node, getVariableID(node), *ip);
    }

    virtual void printLoopIndexedTmp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedTmp, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for loop indexed temporary operation")
        Node* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")

        print(node.getArguments()[1]);
    }

    virtual void printTmpVar(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Tmp, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for temporary variable usage operation")
        Node* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")

        _code << _startVar << *tmpVar->getName() << _endVar;
    }

    virtual void printIndexAssign(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexAssign, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for an index assignment operation")

        auto& inode = static_cast<IndexAssignOperationNode<Base>&> (node);

        checkEquationEnvStart();

        const IndexPattern& ip = inode.getIndexPattern();
        _code << _startAlgLine << _startEq
                << (*inode.getIndex().getName())
                << _assignStr << indexPattern2String(ip, inode.getIndexPatternIndexes())
                << _endEq << _endAlgLine << _endline;
    }

    virtual void printIndexCondExprOp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexCondExpr, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for an index condition expression operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an index condition expression operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation()->getOperationType() == CGOpCode::Index, "Invalid argument for an index condition expression operation")

        const std::vector<size_t>& info = node.getInfo();

        auto& iterationIndexOp = static_cast<IndexOperationNode<Base>&> (*node.getArguments()[0].getOperation());
        const std::string& index = *iterationIndexOp.getIndex().getName();

        checkEquationEnvStart();

        printIndexCondExpr(_code, info, index);
    }

    virtual void printStartIf(Node& node) {
        /**
         * the first argument is the condition, following arguments are
         * just extra dependencies that must be defined outside the if
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::StartIf, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'if start' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'if start' operation")

        checkEquationEnvEnd();

        _code << _ifStart;
        //checkEquationEnvStart(); // no need
        _code << _conditionStart;
        printIndexCondExprOp(*node.getArguments()[0].getOperation());
        checkEquationEnvEnd();
        _code << _conditionEnd;
        _code << _endline;

        _indentationLevel++;
    }

    virtual void printElseIf(Node& node) {
        /**
         * the first argument is the condition, the second argument is the
         * if start node, the following arguments are assignments in the
         * previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::ElseIf, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 2, "Invalid number of arguments for an 'else if' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'else if' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an 'else if' operation")

        checkEquationEnvEnd();
        _indentationLevel--;

        // close previous environment
        CGOpCode nType = node.getArguments()[0].getOperation()->getOperationType();
        if (nType == CGOpCode::StartIf) {
            _code << _ifEnd << _endline;
        } else if (nType == CGOpCode::ElseIf) {
            _code << _elseIfEnd << _endline;
        }

        // start new else if
        _code << _elseIfStart;
        _code << _conditionStart;
        //checkEquationEnvStart(); // no need
        printIndexCondExprOp(*node.getArguments()[1].getOperation());
        checkEquationEnvEnd();
        _code << _conditionEnd;
        _code << _endline;

        _indentationLevel++;
    }

    virtual void printElse(Node& node) {
        /**
         * the first argument is the  if start node, the following arguments
         * are assignments in the previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Else, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'else' operation")

        checkEquationEnvEnd();
        _indentationLevel--;

        // close previous environment
        CGOpCode nType = node.getArguments()[0].getOperation()->getOperationType();
        if (nType == CGOpCode::StartIf) {
            _code << _ifEnd << _endline;
        } else if (nType == CGOpCode::ElseIf) {
            _code << _elseIfEnd << _endline;
        }

        // start else
        _code << _elseStart << _endline;

        _indentationLevel++;
    }

    virtual void printEndIf(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::EndIf, "Invalid node type for an 'end if' operation")

        _indentationLevel--;

        // close previous environment
        CGOpCode nType = node.getArguments()[0].getOperation()->getOperationType();
        if (nType == CGOpCode::StartIf) {
            _code << _ifEnd << _endline;
        } else if (nType == CGOpCode::ElseIf) {
            _code << _elseIfEnd << _endline;
        } else {
            assert(nType == CGOpCode::Else);
            _code << _elseEnd << _endline;
        }
    }

    virtual void printCondResult(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::CondResult, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for an assignment inside an if/else operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation")

        // just follow the argument
        Node& nodeArg = *node.getArguments()[1].getOperation();
        printAssignment(nodeArg);
    }

    virtual void printUserCustom(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::UserCustom, "Invalid node type")

        throw CGException("Unable to generate Latex for user custom operation nodes.");
    }

    inline bool isDependent(const Node& arg) const {
        if (arg.getOperationType() == CGOpCode::LoopIndexedDep) {
            return true;
        }
        size_t id = getVariableID(arg);
        return id > _independentSize && id < _minTemporaryVarID;
    }

    virtual void printParameter(const Base& value) {
        // make sure all digits of floating point values are printed
        std::ostringstream os;
        os << std::setprecision(_parameterPrecision) << value;

        std::string number = os.str();
        size_t pos = number.find('e');
        if (pos != std::string::npos) {
            std::string n = " \\times 10^{";
            number.replace(pos, 1, n);
            pos += n.size();
            if (number[pos] == '-' || number[pos] == '+')
                pos++;
            while (number[pos] == '0')
                number.replace(pos, 1, ""); // remove zeros

            number += "}";
        }
        _code << number;

    }

    virtual const std::string& getComparison(enum CGOpCode op) const {
        switch (op) {
            case CGOpCode::ComLt:
                return _COMP_OP_LT;

            case CGOpCode::ComLe:
                return _COMP_OP_LE;

            case CGOpCode::ComEq:
                return _COMP_OP_EQ;

            case CGOpCode::ComGe:
                return _COMP_OP_GE;

            case CGOpCode::ComGt:
                return _COMP_OP_GT;

            case CGOpCode::ComNe:
                return _COMP_OP_NE;

            default:
                CPPAD_ASSERT_UNKNOWN(0)
                break;
        }
        throw CGException("Invalid comparison operator code"); // should never get here
    }

    inline const std::string& getPrintfBaseFormat() {
        static const std::string format; // empty string
        return format;
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
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
            case CGOpCode::Erfc:
            case CGOpCode::Asinh:
            case CGOpCode::Acosh:
            case CGOpCode::Atanh:
            case CGOpCode::Expm1:
            case CGOpCode::Log1p:
#endif
            case CGOpCode::Sqrt:
            case CGOpCode::Tanh:
            case CGOpCode::Tan:
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
const std::string LanguageLatex<Base>::_COMP_OP_LT = "<"; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageLatex<Base>::_COMP_OP_LE = "\\le"; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageLatex<Base>::_COMP_OP_EQ = "=="; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageLatex<Base>::_COMP_OP_GE = "\\ge"; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageLatex<Base>::_COMP_OP_GT = ">"; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageLatex<Base>::_COMP_OP_NE = "\\ne"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageLatex<Base>::_C_STATIC_INDEX_ARRAY = "index"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageLatex<Base>::_C_SPARSE_INDEX_ARRAY = "idx"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageLatex<Base>::_ATOMIC_TX = "atx"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageLatex<Base>::_ATOMIC_TY = "aty"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageLatex<Base>::_ATOMIC_PX = "apx"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageLatex<Base>::_ATOMIC_PY = "apy"; // NOLINT(cert-err58-cpp)

} // END cg namespace
} // END CppAD namespace

#endif
