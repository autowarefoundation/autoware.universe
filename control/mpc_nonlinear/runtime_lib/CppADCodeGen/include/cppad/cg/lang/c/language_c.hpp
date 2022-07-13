#ifndef CPPAD_CG_LANGUAGE_C_INCLUDED
#define CPPAD_CG_LANGUAGE_C_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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

#define CPPAD_CG_C_LANG_FUNCNAME(fn) \
inline virtual const std::string& fn ## FuncName() {\
    static const std::string name(#fn);\
    return name;\
}

namespace CppAD {
namespace cg {

/**
 * Generates code for the C language
 *
 * @author Joao Leal
 */
template<class Base>
class LanguageC : public Language<Base> {
public:
    using Node = OperationNode<Base>;
    using Arg = Argument<Base>;
public:
    static const std::string U_INDEX_TYPE;
    static const std::string ATOMICFUN_STRUCT_DEFINITION;
protected:
    static const std::string _C_COMP_OP_LT;
    static const std::string _C_COMP_OP_LE;
    static const std::string _C_COMP_OP_EQ;
    static const std::string _C_COMP_OP_GE;
    static const std::string _C_COMP_OP_GT;
    static const std::string _C_COMP_OP_NE;
    static const std::string _C_STATIC_INDEX_ARRAY;
    static const std::string _C_SPARSE_INDEX_ARRAY;
    static const std::string _ATOMIC_TX;
    static const std::string _ATOMIC_TY;
    static const std::string _ATOMIC_PX;
    static const std::string _ATOMIC_PY;
private:
    class AtomicFuncArray; //forward declaration
protected:
    // the type name of the Base class (e.g. "double")
    const std::string _baseTypeName;
    // spaces for 1 level indentation
    const std::string _spaces;
    // information from the code handler (not owned)
    std::unique_ptr<LanguageGenerationData<Base>> _info;
    // current indentation
    std::string _indentation;
    // variable name used for the inlet variable
    std::string _inArgName;
    // variable name used for the outlet variable
    std::string _outArgName;
    // variable name used for the atomic functions array
    std::string _atomicArgName;
    // output stream for the generated source code
    std::ostringstream _code;
    // creates the variable names
    VariableNameGenerator<Base>* _nameGen;
    // auxiliary string stream
    std::ostringstream _ss;
    //
    LangStreamStack<Base> _streamStack;
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
    // the operator used for assignment of dependent variables
    std::string _depAssignOperation;
    // whether or not to ignore assignment of constant zero values to dependent variables
    bool _ignoreZeroDepAssign;
    // the name of the function to be created (if the string is empty no function is created)
    std::string _functionName;
    // the maximum number of assignments (~lines) per local function
    size_t _maxAssignmentsPerFunction;
    // the maximum number of operations per variable assignment
    size_t _maxOperationsPerAssignment;
    //  maps file names to with their contents
    std::map<std::string, std::string>* _sources;
    // the values in the temporary array
    std::vector<const Arg*> _tmpArrayValues;
    // the values in the temporary sparse array
    std::vector<const Arg*> _tmpSparseArrayValues;
    // the current state of Array structures used by atomic functions
    std::map<std::string, AtomicFuncArray> _atomicFuncArrays;
    // indexes defined as function arguments
    std::vector<const Node*> _funcArgIndexes;
    std::vector<const LoopStartOperationNode<Base>*> _currentLoops;
    // the maximum precision used to print values
    size_t _parameterPrecision;
private:
    std::vector<std::string> funcArgDcl_;
    std::vector<std::string> localFuncArgDcl_;
    std::string localFuncArgs_;
    std::string auxArrayName_;

public:

    /**
     * Creates a C language source code generator
     *
     * @param varTypeName variable data type (e.g. double)
     * @param spaces number of spaces for indentations
     */
    explicit LanguageC(std::string varTypeName,
                       size_t spaces = 3) :
        _baseTypeName(std::move(varTypeName)),
        _spaces(spaces, ' '),
        _info(nullptr),
        _inArgName("in"),
        _outArgName("out"),
        _atomicArgName("atomicFun"),
        _nameGen(nullptr),
        _streamStack(_code),
        _independentSize(0), // not really required (but it avoids warnings)
        _minTemporaryVarID(0), // not really required (but it avoids warnings)
        _dependent(nullptr),
        _depAssignOperation("="),
        _ignoreZeroDepAssign(false),
        _maxAssignmentsPerFunction(0),
        _maxOperationsPerAssignment((std::numeric_limits<size_t>::max)()),
        _sources(nullptr),
        _parameterPrecision(std::numeric_limits<Base>::digits10) {
    }

    inline virtual ~LanguageC() = default;

    inline const std::string& getArgumentIn() const {
        return _inArgName;
    }

    inline void setArgumentIn(const std::string& inArgName) {
        _inArgName = inArgName;
    }

    inline const std::string& getArgumentOut() const {
        return _outArgName;
    }

    inline void setArgumentOut(const std::string& outArgName) {
        _outArgName = outArgName;
    }

    inline const std::string& getArgumentAtomic() const {
        return _atomicArgName;
    }

    inline void setArgumentAtomic(const std::string& atomicArgName) {
        _atomicArgName = atomicArgName;
    }

    inline const std::string& getDependentAssignOperation() const {
        return _depAssignOperation;
    }

    inline void setDependentAssignOperation(const std::string& depAssignOperation) {
        _depAssignOperation = depAssignOperation;
    }

    /**
     * Whether or not source code to set dependent variables to zero will be generated.
     * It may be used not to set the dependent variables to zero when it is known
     * they are already set to zero before the source code generation.
     *
     * @return true if source code to explicitly set dependent variables to zero will NOT be created.
     */
    inline bool isIgnoreZeroDepAssign() const {
        return _ignoreZeroDepAssign;
    }

    /**
     * Whether or not to generate expressions to set dependent variables to zero.
     * It may be used not to set the dependent variables to zero when it is known
     * they are already set to zero before the source code generation.
     *
     * @param ignore true if source code to explicitly set dependent variables to zero will NOT be created.
     */
    inline void setIgnoreZeroDepAssign(bool ignore) {
        _ignoreZeroDepAssign = ignore;
    }

    virtual void setGenerateFunction(const std::string& functionName) {
        _functionName = functionName;
    }

    virtual void setFunctionIndexArgument(const Node& funcArgIndex) {
        _funcArgIndexes.resize(1);
        _funcArgIndexes[0] = &funcArgIndex;
    }

    virtual void setFunctionIndexArguments(const std::vector<const Node*>& funcArgIndexes) {
        _funcArgIndexes = funcArgIndexes;
    }

    virtual const std::vector<const Node*>& getFunctionIndexArguments() const {
        return _funcArgIndexes;
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
     * Defines the maximum number of assignment per generated function.
     * Zero means it is disabled (no limit).
     * By setting a limit, it is possible to reduce the compiler workload by having multiple file/function
     * instead of a very large one.
     * Note that it is not possible to split some function (e.g., containing loops) and, therefore, this
     * limit can be violated.
     *
     * @param maxAssignmentsPerFunction the maximum number of assignments per file/function
     * @param sources A map where the file names are associated with their contents.
     */
    virtual void setMaxAssignmentsPerFunction(size_t maxAssignmentsPerFunction,
                                              std::map<std::string, std::string>* sources) {
        _maxAssignmentsPerFunction = maxAssignmentsPerFunction;
        _sources = sources;
    }

    /**
     * The maximum number of operations per variable assignment.
     *
     * @return The maximum number of operations per variable assignment
     */
    inline size_t getMaxOperationsPerAssignment() const {
        return _maxOperationsPerAssignment;
    }

    /**
     * Defines the maximum number of operations per variable assignment.
     * Defining a limit can reduce the memory required for compilation of the source code.
     *
     * @param maxOperationsPerAssignment  The maximum number of operations per variable assignment.
     */
    inline void setMaxOperationsPerAssignment(size_t maxOperationsPerAssignment) {
        _maxOperationsPerAssignment = maxOperationsPerAssignment;
    }

    inline std::string generateTemporaryVariableDeclaration(bool isWrapperFunction,
                                                            bool zeroArrayDependents,
                                                            const std::vector<int>& atomicMaxForward,
                                                            const std::vector<int>& atomicMaxReverse) {
        int maxForward = -1;
        if (!atomicMaxForward.empty())
            maxForward = *std::max_element(atomicMaxForward.begin(), atomicMaxForward.end());

        int maxReverse = -1;
        if (!atomicMaxReverse.empty())
            maxReverse = *std::max_element(atomicMaxReverse.begin(), atomicMaxReverse.end());

        return generateTemporaryVariableDeclaration(isWrapperFunction, zeroArrayDependents,
                                                    maxForward, maxReverse);
    }

    /**
     * Declares temporary variables used by a function.
     *
     * @param isWrapperFunction true if the declarations are for a wrapper
     *                               function which calls other functions
     *                               where the actual work is performed
     * @param zeroArrayDependents  whether or not the dependent variables
     *                             should be set to zero before executing
     *                             the operation graph
     * @param maxForwardOrder the maximum order of forward mode calls to
     *                        atomic functions
     * @param maxReverseOrder the maximum order of reverse mode calls to
     *                        atomic functions
     * @return the string with the declarations for the temporary variables
     */
    virtual std::string generateTemporaryVariableDeclaration(bool isWrapperFunction = false,
                                                             bool zeroArrayDependents = false,
                                                             int maxForwardOrder = -1,
                                                             int maxReverseOrder = -1) {
        CPPADCG_ASSERT_UNKNOWN(_nameGen != nullptr);

        // declare variables
        const std::vector<FuncArgument>& tmpArg = _nameGen->getTemporary();

        CPPADCG_ASSERT_KNOWN(tmpArg.size() == 3,
                             "There must be two temporary variables")

        _ss << _spaces << "// auxiliary variables\n";
        /**
         * temporary variables
         */
        if (tmpArg[0].array) {
            size_t size = _nameGen->getMaxTemporaryVariableID() + 1 - _nameGen->getMinTemporaryVariableID();
            if (size > 0 || isWrapperFunction) {
                _ss << _spaces << _baseTypeName << " " << tmpArg[0].name << "[" << size << "];\n";
            }
        } else if (_temporary.size() > 0) {
            for (const std::pair<size_t, Node*>& p : _temporary) {
                Node* var = p.second;
                if (var->getName() == nullptr) {
                    var->setName(_nameGen->generateTemporary(*var, getVariableID(*var)));
                }
            }

            Node* var1 = _temporary.begin()->second;
            const std::string& varName1 = *var1->getName();
            _ss << _spaces << _baseTypeName << " " << varName1;

            typename std::map<size_t, Node*>::const_iterator it = _temporary.begin();
            for (it++; it != _temporary.end(); ++it) {
                _ss << ", " << *it->second->getName();
            }
            _ss << ";\n";
        }

        /**
         * temporary array variables
         */
        size_t arraySize = _nameGen->getMaxTemporaryArrayVariableID();
        if (arraySize > 0 || isWrapperFunction) {
            _ss << _spaces << _baseTypeName << " " << tmpArg[1].name << "[" << arraySize << "];\n";
        }

        /**
         * temporary sparse array variables
         */
        size_t sArraySize = _nameGen->getMaxTemporarySparseArrayVariableID();
        if (sArraySize > 0 || isWrapperFunction) {
            _ss << _spaces << _baseTypeName << " " << tmpArg[2].name << "[" << sArraySize << "];\n";
            _ss << _spaces << U_INDEX_TYPE << " " << _C_SPARSE_INDEX_ARRAY << "[" << sArraySize << "];\n";
        }

        if (!isWrapperFunction) {
            generateArrayContainersDeclaration(_ss, maxForwardOrder, maxReverseOrder);
        }

        //
        if (!isWrapperFunction && (arraySize > 0 || sArraySize > 0)) {
            _ss << _spaces << _baseTypeName << "* " << auxArrayName_ << ";\n";
        }

        if ((isWrapperFunction && zeroArrayDependents) ||
                (!isWrapperFunction && (arraySize > 0 || sArraySize > 0 || zeroArrayDependents))) {
            _ss << _spaces << U_INDEX_TYPE << " i;\n";
        }

        // loop indexes
        createIndexDeclaration();

        // clean-up
        std::string code = _ss.str();
        _ss.str("");

        return code;
    }

    inline void generateArrayContainersDeclaration(std::ostringstream& ss,
                                                   const std::vector<int>& atomicMaxForward,
                                                   const std::vector<int>& atomicMaxReverse) {
        int maxForward = -1;
        if (!atomicMaxForward.empty())
            maxForward = *std::max_element(atomicMaxForward.begin(), atomicMaxForward.end());

        int maxReverse = -1;
        if (!atomicMaxReverse.empty())
            maxReverse = *std::max_element(atomicMaxReverse.begin(), atomicMaxReverse.end());

        generateArrayContainersDeclaration(ss, maxForward, maxReverse);
    }

    virtual void generateArrayContainersDeclaration(std::ostringstream& ss,
                                                    int maxForwardOrder = -1,
                                                    int maxReverseOrder = -1) {
        if (maxForwardOrder >= 0 || maxReverseOrder >= 0) {
            ss << _spaces << "Array " << _ATOMIC_TX << "[" << (std::max<int>(maxForwardOrder, maxReverseOrder) + 1) << "];\n";
            if (maxForwardOrder >= 0)
                ss << _spaces << "Array " << _ATOMIC_TY << ";\n";
            if (maxReverseOrder >= 0) {
                ss << _spaces << "Array " << _ATOMIC_PX << ";\n";
                ss << _spaces << "Array " << _ATOMIC_PY << "[" << (maxReverseOrder + 1) << "];\n";
            }
        }
    }

    virtual std::string generateDependentVariableDeclaration() {
        const std::vector<FuncArgument>& depArg = _nameGen->getDependent();
        CPPADCG_ASSERT_KNOWN(!depArg.empty(),
                             "There must be at least one dependent argument")

        _ss << _spaces << "//dependent variables\n";
        for (size_t i = 0; i < depArg.size(); i++) {
            _ss << _spaces << argumentDeclaration(depArg[i]) << " = " << _outArgName << "[" << i << "];\n";
        }

        std::string code = _ss.str();
        _ss.str("");
        return code;
    }

    virtual std::string generateIndependentVariableDeclaration() {
        const std::vector<FuncArgument>& indArg = _nameGen->getIndependent();
        CPPADCG_ASSERT_KNOWN(!indArg.empty(),
                             "There must be at least one independent argument")

        _ss << _spaces << "//independent variables\n";
        for (size_t i = 0; i < indArg.size(); i++) {
            _ss << _spaces << "const " << argumentDeclaration(indArg[i]) << " = " << _inArgName << "[" << i << "];\n";
        }

        std::string code = _ss.str();
        _ss.str("");
        return code;
    }

    inline std::string generateArgumentAtomicDcl() const {
        return "struct LangCAtomicFun " + _atomicArgName;
    }

    virtual std::string generateFunctionArgumentsDcl() const {
        std::string args = generateFunctionIndexArgumentsDcl();
        if (!args.empty())
            args += ", ";
        args += generateDefaultFunctionArgumentsDcl();

        return args;
    }

    virtual std::vector<std::string> generateFunctionArgumentsDcl2() const {
        std::vector<std::string> args = generateFunctionIndexArgumentsDcl2();
        std::vector<std::string> dArgs = generateDefaultFunctionArgumentsDcl2();
        args.insert(args.end(), dArgs.begin(), dArgs.end());
        return args;
    }

    virtual std::string generateDefaultFunctionArgumentsDcl() const {
        return implode(generateDefaultFunctionArgumentsDcl2(), ", ");
    }

    virtual std::vector<std::string> generateDefaultFunctionArgumentsDcl2() const {
        return std::vector<std::string> {_baseTypeName + " const *const * " + _inArgName,
                                         _baseTypeName + "*const * " + _outArgName,
                                         generateArgumentAtomicDcl()};
    }

    virtual std::string generateFunctionIndexArgumentsDcl() const {
        return implode(generateFunctionIndexArgumentsDcl2(), ", ");
    }

    virtual std::vector<std::string> generateFunctionIndexArgumentsDcl2() const {
        std::vector<std::string> argtxt(_funcArgIndexes.size());
        for (size_t a = 0; a < _funcArgIndexes.size(); a++) {
            argtxt[a] = U_INDEX_TYPE + " " + *_funcArgIndexes[a]->getName();
        }
        return argtxt;
    }

    virtual std::string generateDefaultFunctionArguments() const {
        return _inArgName + ", " + _outArgName + ", " + _atomicArgName;
    }

    virtual std::string generateFunctionIndexArguments() const {
        std::string argtxt;
        for (size_t a = 0; a < _funcArgIndexes.size(); a++) {
            if (a > 0) argtxt += ", ";
            argtxt += *_funcArgIndexes[a]->getName();
        }
        return argtxt;
    }

    inline void createIndexDeclaration();

    CPPAD_CG_C_LANG_FUNCNAME(abs)
    CPPAD_CG_C_LANG_FUNCNAME(acos)
    CPPAD_CG_C_LANG_FUNCNAME(asin)
    CPPAD_CG_C_LANG_FUNCNAME(atan)
    CPPAD_CG_C_LANG_FUNCNAME(cosh)
    CPPAD_CG_C_LANG_FUNCNAME(cos)
    CPPAD_CG_C_LANG_FUNCNAME(exp)
    CPPAD_CG_C_LANG_FUNCNAME(log)
    CPPAD_CG_C_LANG_FUNCNAME(sinh)
    CPPAD_CG_C_LANG_FUNCNAME(sin)
    CPPAD_CG_C_LANG_FUNCNAME(sqrt)
    CPPAD_CG_C_LANG_FUNCNAME(tanh)
    CPPAD_CG_C_LANG_FUNCNAME(tan)
    CPPAD_CG_C_LANG_FUNCNAME(pow)

#if CPPAD_USE_CPLUSPLUS_2011
    CPPAD_CG_C_LANG_FUNCNAME(erf)
    CPPAD_CG_C_LANG_FUNCNAME(erfc)
    CPPAD_CG_C_LANG_FUNCNAME(asinh)
    CPPAD_CG_C_LANG_FUNCNAME(acosh)
    CPPAD_CG_C_LANG_FUNCNAME(atanh)
    CPPAD_CG_C_LANG_FUNCNAME(expm1)
    CPPAD_CG_C_LANG_FUNCNAME(log1p)
#endif

    /**
     * Prints a function declaration where each argument is in a different line.
     *
     * @param out the stream where the declaration is printed
     * @param returnType the function return type
     * @param functionName the function name
     * @param arguments function arguments
     * @param arguments2 additional function arguments
     */
    static inline void printFunctionDeclaration(std::ostringstream& out,
                                                const std::string& returnType,
                                                const std::string& functionName,
                                                const std::vector<std::string>& arguments,
                                                const std::vector<std::string>& arguments2 = {}) {
        out << returnType << " " << functionName << "(";
        size_t i = 0;
        size_t offset = returnType.size() + 1 + functionName.size() + 1;
        for (const std::string& a : arguments) {
            if (i > 0) {
                out << ",\n" << std::setw(offset) << " ";
            }
            out << a;
            ++i;
        }
        for (const std::string& a : arguments2) {
            if (i > 0) {
                out << ",\n" << std::setw(offset) << " ";
            }
            out << a;
            ++i;
        }
        out << ")";
    }

    static inline void printIndexCondExpr(std::ostringstream& out,
                                          const std::vector<size_t>& info,
                                          const std::string& index) {
        CPPADCG_ASSERT_KNOWN(info.size() > 1 && info.size() % 2 == 0, "Invalid number of information elements for an index condition expression operation")

        size_t infoSize = info.size();
        for (size_t e = 0; e < infoSize; e += 2) {
            if (e > 0) {
                out << " || ";
            }
            size_t min = info[e];
            size_t max = info[e + 1];
            if (min == max) {
                out << index << " == " << min;
            } else if (min == 0) {
                out << index << " <= " << max;
            } else if (max == (std::numeric_limits<size_t>::max)()) {
                out << min << " <= " << index;
            } else {
                if (infoSize != 2)
                    out << "(";

                if (max - min == 1)
                    out << min << " == " << index << " || " << index << " == " << max;
                else
                    out << min << " <= " << index << " && " << index << " <= " << max;

                if (infoSize != 2)
                    out << ")";
            }
        }
    }

    /***********************************************************************
     *
     **********************************************************************/

    static inline void printStaticIndexArray(std::ostringstream& os,
                                             const std::string& name,
                                             const std::vector<size_t>& values);

    static inline void printStaticIndexMatrix(std::ostringstream& os,
                                              const std::string& name,
                                              const std::map<size_t, std::map<size_t, size_t> >& values);

    /***********************************************************************
     * index patterns
     **********************************************************************/
    static inline void generateNames4RandomIndexPatterns(const std::set<RandomIndexPattern*>& randomPatterns);

    static inline void printRandomIndexPatternDeclaration(std::ostringstream& os,
                                                          const std::string& identation,
                                                          const std::set<RandomIndexPattern*>& randomPatterns);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const Node& index);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::string& index);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::vector<const Node*>& indexes);

    static inline std::string indexPattern2String(const IndexPattern& ip,
                                                  const std::vector<const std::string*>& indexes);

    static inline std::string linearIndexPattern2String(const LinearIndexPattern& lip,
                                                        const Node& index);

    static inline std::string linearIndexPattern2String(const LinearIndexPattern& lip,
                                                        const std::string& index);

    static inline bool isOffsetBy(const IndexPattern* ip,
                                  const IndexPattern* refIp,
                                  long offset);

    static inline bool isOffsetBy(const LinearIndexPattern* lIp,
                                  const LinearIndexPattern* refLIp,
                                  long offset);

    static inline bool isOffsetBy(const LinearIndexPattern& lIp,
                                  const LinearIndexPattern& refLIp,
                                  long offset);


    static inline bool isOffsetBy(const SectionedIndexPattern* sIp,
                                  const SectionedIndexPattern* refSecp,
                                  long offset);

    static inline bool isOffsetBy(const SectionedIndexPattern& lIp,
                                  const SectionedIndexPattern& refSecp,
                                  long offset);

    static inline Plane2DIndexPattern* encapsulateIndexPattern(const LinearIndexPattern& refLIp,
                                                               size_t starti);

    static inline Plane2DIndexPattern* encapsulateIndexPattern(const SectionedIndexPattern& refSecp,
                                                               size_t starti);
protected:

    void generateSourceCode(std::ostream& out,
                            std::unique_ptr<LanguageGenerationData<Base> > info) override {

        const bool createFunction = !_functionName.empty();
        const bool multiFunction = createFunction && _maxAssignmentsPerFunction > 0 && _sources != nullptr;

        // clean up
        _code.str("");
        _ss.str("");
        _temporary.clear();
        _indentation = _spaces;
        funcArgDcl_.clear();
        localFuncArgDcl_.clear();
        localFuncArgs_ = "";
        auxArrayName_ = "";
        _currentLoops.clear();
        _atomicFuncArrays.clear();
        _streamStack.clear();
        _dependentIDs.clear();

        // save some info
        _info = std::move(info);
        _independentSize = _info->independent.size();
        _dependent = &_info->dependent;
        _nameGen = &_info->nameGen;
        _minTemporaryVarID = _info->minTemporaryVarID;
        const ArrayView<CG<Base> >& dependent = _info->dependent;
        const std::vector<Node*>& variableOrder = _info->variableOrder;

        _tmpArrayValues.resize(_nameGen->getMaxTemporaryArrayVariableID());
        std::fill(_tmpArrayValues.begin(), _tmpArrayValues.end(), nullptr);
        _tmpSparseArrayValues.resize(_nameGen->getMaxTemporarySparseArrayVariableID());
        std::fill(_tmpSparseArrayValues.begin(), _tmpSparseArrayValues.end(), nullptr);

        /**
         * generate index array names (might be used for variable names)
         */
        generateNames4RandomIndexPatterns(_info->indexRandomPatterns);

        /**
         * generate variable names
         */
        //generate names for the independent variables
        for (size_t j = 0; j < _independentSize; j++) {
            Node& op = *_info->independent[j];
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
                    const IndexPattern* ip = _info->loopDependentIndexPatterns[pos];
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

        if (createFunction) {
            funcArgDcl_ = generateFunctionArgumentsDcl2();

            localFuncArgDcl_.reserve(funcArgDcl_.size() + 4);
            localFuncArgDcl_ = funcArgDcl_;
            localFuncArgDcl_.push_back(argumentDeclaration(tmpArg[0]));
            localFuncArgDcl_.push_back(argumentDeclaration(tmpArg[1]));
            localFuncArgDcl_.push_back(argumentDeclaration(tmpArg[2]));
            localFuncArgDcl_.push_back(U_INDEX_TYPE + "* " + _C_SPARSE_INDEX_ARRAY);

            localFuncArgs_ = generateDefaultFunctionArguments() + ", "
                    + tmpArg[0].name + ", "
                    + tmpArg[1].name + ", "
                    + tmpArg[2].name + ", "
                    + _C_SPARSE_INDEX_ARRAY;
        }

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
        std::vector<std::string> localFuncNames;
        if (multiFunction) {
            localFuncNames.reserve(variableOrder.size() / _maxAssignmentsPerFunction);
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
            if (_info->zeroDependents) {
                // zero initial values
                for (size_t i = 0; i < depArg.size(); i++) {
                    const FuncArgument& a = depArg[i];
                    if (a.array) {
                        _code << _indentation << "for(i = 0; i < " << _dependent->size() << "; i++) " << a.name << "[i]";
                    } else {
                        _code << _indentation << _nameGen->generateDependent(i);
                    }
                    _code << " = ";
                    printParameter(Base(0.0));
                    _code << ";\n";
                }
            }

            size_t assignCount = 0;
            for (size_t i = 0; i < variableOrder.size(); ++i) {
                Node* it = variableOrder[i];

                // check if a new function should start
                if (assignCount >= _maxAssignmentsPerFunction && multiFunction && _currentLoops.empty()) {
                    assignCount = 0;
                    saveLocalFunction(localFuncNames, localFuncNames.empty() && _info->zeroDependents);
                }

                Node& node = *it;

                // a dependent variable assigned by a loop does require any source code (its done inside the loop)
                if (node.getOperationType() == CGOpCode::DependentRefRhs) {
                    continue; // nothing to do (this operation is right hand side only)
                } else if (node.getOperationType() == CGOpCode::TmpDcl) { // temporary variable declaration does not need any source code here
                    continue; // nothing to do (bogus operation)
                } else if (node.getOperationType() == CGOpCode::LoopIndexedDep) {
                    // try to detect a pattern and use a loop instead of individual assignments
                    i = printLoopIndexDeps(variableOrder, i);
                    continue;
                }

                assignCount += printAssignment(node);
                
                CPPAD_ASSERT_KNOWN(_streamStack.empty(), "Error writing all operations to output stream")
            }

            if (!localFuncNames.empty() && assignCount > 0) {
                assignCount = 0;
                saveLocalFunction(localFuncNames, false);
            }
        }

        if (!localFuncNames.empty()) {
            /**
             * Create the wrapper function which calls the other functions
             */
            CPPADCG_ASSERT_KNOWN(tmpArg[0].array,
                                 "The temporary variables must be saved in an array in order to generate multiple functions")

            _code << ATOMICFUN_STRUCT_DEFINITION << "\n\n";
            // forward declarations
            std::string localFuncArgDcl2 = implode(localFuncArgDcl_, ", ");
            for (auto & localFuncName : localFuncNames) {
                _code << "void " << localFuncName << "(" << localFuncArgDcl2 << ");\n";
            }
            _code << "\n";
            printFunctionDeclaration(_code, "void", _functionName, funcArgDcl_);
            _code  << " {\n";
            _nameGen->customFunctionVariableDeclarations(_code);
            _code << generateIndependentVariableDeclaration() << "\n";
            _code << generateDependentVariableDeclaration() << "\n";
            _code << generateTemporaryVariableDeclaration(true, false,
                                                          _info->atomicFunctionsMaxForward,
                                                          _info->atomicFunctionsMaxReverse) << "\n";
            _nameGen->prepareCustomFunctionVariables(_code);
            for (auto & localFuncName : localFuncNames) {
                _code << _spaces << localFuncName << "(" << localFuncArgs_ << ");\n";
            }
        }

        // dependent duplicates
        if (!dependentDuplicates.empty()) {
            _code << _spaces << "// variable duplicates: " << dependentDuplicates.size() << "\n";
            for (size_t index : dependentDuplicates) {
                const CG<Base>& dep = (*_dependent)[index];
                std::string varName = _nameGen->generateDependent(index);
                const std::string& origVarName = *dep.getOperationNode()->getName();

                _code << _spaces << varName << " " << _depAssignOperation << " " << origVarName << ";\n";
            }
        }

        // constant dependent variables
        bool commentWritten = false;
        for (size_t i = 0; i < dependent.size(); i++) {
            if (dependent[i].isParameter()) {
                if (!_ignoreZeroDepAssign || !dependent[i].isIdenticalZero()) {
                    if (!commentWritten) {
                        _code << _spaces << "// dependent variables without operations\n";
                        commentWritten = true;
                    }
                    std::string varName = _nameGen->generateDependent(i);
                    _code << _spaces << varName << " " << _depAssignOperation << " ";
                    printParameter(dependent[i].getValue());
                    _code << ";\n";
                }
            } else if (dependent[i].getOperationNode()->getOperationType() == CGOpCode::Inv) {
                if (!commentWritten) {
                    _code << _spaces << "// dependent variables without operations\n";
                    commentWritten = true;
                }
                std::string varName = _nameGen->generateDependent(i);
                const std::string& indepName = *dependent[i].getOperationNode()->getName();
                _code << _spaces << varName << " " << _depAssignOperation << " " << indepName << ";\n";
            }
        }

        /**
         * encapsulate the code in a function
         */
        if (createFunction) {
            if (localFuncNames.empty()) {
                _ss << "#include <math.h>\n"
                        "#include <stdio.h>\n\n"
                    << ATOMICFUN_STRUCT_DEFINITION << "\n\n";
                printFunctionDeclaration(_ss, "void", _functionName, funcArgDcl_);
                _ss << " {\n";
                _nameGen->customFunctionVariableDeclarations(_ss);
                _ss << generateIndependentVariableDeclaration() << "\n";
                _ss << generateDependentVariableDeclaration() << "\n";
                _ss << generateTemporaryVariableDeclaration(false, _info->zeroDependents,
                                                            _info->atomicFunctionsMaxForward,
                                                            _info->atomicFunctionsMaxReverse) << "\n";
                _nameGen->prepareCustomFunctionVariables(_ss);
                _ss << _code.str();
                _nameGen->finalizeCustomFunctionVariables(_ss);
                _ss << "}\n\n";

                out << _ss.str();

                if (_sources != nullptr) {
                    (*_sources)[_functionName + ".c"] = _ss.str();
                }
            } else {
                _nameGen->finalizeCustomFunctionVariables(_code);
                _code << "}\n\n";

                (*_sources)[_functionName + ".c"] = _code.str();
            }
        } else {
            out << _code.str();
        }
    }

    inline size_t getVariableID(const Node& node) const {
        return _info->varId[node];
    }

    inline unsigned printAssignment(Node& node) {
        return pushAssignment(node, node);
    }

    inline unsigned pushAssignment(Node& nodeName,
                                   const Arg& nodeRhs) {
        if (nodeRhs.getOperation() != nullptr) {
            return pushAssignment(nodeName, *nodeRhs.getOperation());
        } else {
            pushAssignmentStart(nodeName);
            pushParameter(*nodeRhs.getParameter());
            pushAssignmentEnd(nodeName);

            _streamStack.flush();

            return 1;
        }
    }

    inline unsigned pushAssignment(Node& nodeName,
                                   Node& nodeRhs) {
        bool createsVar = directlyAssignsVariable(nodeRhs); // do we need to do the assignment here?
        if (!createsVar) {
            pushAssignmentStart(nodeName);
        }
        unsigned lines = pushExpressionNoVarCheck2(nodeRhs);
        if (!createsVar) {
            pushAssignmentEnd(nodeRhs);
        }

        _streamStack.flush();

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

    inline virtual void pushAssignmentStart(Node& op) {
        pushAssignmentStart(op, createVariableName(op), isDependent(op));
    }

    inline virtual void pushAssignmentStart(Node& node,
                                            const std::string& varName,
                                            bool isDep) {
        if (!isDep) {
            _temporary[getVariableID(node)] = &node;
        }

        _streamStack << _indentation << varName << " ";
        if (isDep) {
            CGOpCode op = node.getOperationType();
            if (op == CGOpCode::DependentMultiAssign || (op == CGOpCode::LoopIndexedDep && node.getInfo()[1] == 1)) {
                _streamStack << "+=";
            } else {
                _streamStack << _depAssignOperation;
            }
        } else {
            _streamStack << "=";
        }
        _streamStack << " ";
    }

    inline virtual void pushAssignmentEnd(Node& op) {
        _streamStack << ";\n";
    }

    virtual std::string argumentDeclaration(const FuncArgument& funcArg) const {
        std::string dcl = _baseTypeName;
        if (funcArg.array) {
            dcl += "*";
        }
        return dcl + " " + funcArg.name;
    }

    virtual void saveLocalFunction(std::vector<std::string>& localFuncNames,
                                   bool zeroDependentArray) {
        _ss << _functionName << "__" << (localFuncNames.size() + 1);
        std::string funcName = _ss.str();
        _ss.str("");

        _ss << "#include <math.h>\n"
                "#include <stdio.h>\n\n"
                << ATOMICFUN_STRUCT_DEFINITION << "\n\n";
        printFunctionDeclaration(_ss, "void", funcName, localFuncArgDcl_);
        _ss << " {\n";
        _nameGen->customFunctionVariableDeclarations(_ss);
        _ss << generateIndependentVariableDeclaration() << "\n";
        _ss << generateDependentVariableDeclaration() << "\n";
        size_t arraySize = _nameGen->getMaxTemporaryArrayVariableID();
        size_t sArraySize = _nameGen->getMaxTemporarySparseArrayVariableID();
        if (arraySize > 0 || sArraySize > 0) {
            _ss << _spaces << _baseTypeName << "* " << auxArrayName_ << ";\n";
        }

        generateArrayContainersDeclaration(_ss,
                                           _info->atomicFunctionsMaxForward,
                                           _info->atomicFunctionsMaxReverse);

        if (arraySize > 0 || sArraySize > 0 || zeroDependentArray) {
            _ss << _spaces << U_INDEX_TYPE << " i;\n";
        }

        // loop indexes
        createIndexDeclaration();

        _nameGen->prepareCustomFunctionVariables(_ss);
        _ss << _code.str();
        _nameGen->finalizeCustomFunctionVariables(_ss);
        _ss << "}\n\n";

        (*_sources)[funcName + ".c"] = _ss.str();
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
                    op == CGOpCode::Assign ||
                    opCount >= _maxOperationsPerAssignment) &&
                    op != CGOpCode::CondResult;
        }
    }

    virtual bool requiresVariableName(const Node& var) const {
        CGOpCode op = var.getOperationType();
        if (_info->totalUseCount.get(var) > 1) {
            return (op != CGOpCode::Pri &&
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
        } else {
            return isCondAssign(op);
        }
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
                op == CGOpCode::Pri ||
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
        return op == CGOpCode::Sign || op == CGOpCode::CondResult || op == CGOpCode::Pri;
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
            } else if (op == CGOpCode::Pri) {
                CPPADCG_ASSERT_KNOWN(var.getArguments().size() == 1, "Invalid number of arguments for print operation")
                Node* tmpVar = var.getArguments()[0].getOperation();
                CPPADCG_ASSERT_KNOWN(tmpVar != nullptr, "Invalid argument for print operation")
                return createVariableName(*tmpVar);

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

    virtual void pushIndependentVariableName(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 0, "Invalid number of arguments for independent variable")

        _streamStack << _nameGen->generateIndependent(op, getVariableID(op));
    }

    virtual unsigned push(const Arg& arg) {
        if (arg.getOperation() != nullptr) {
            // expression
            return pushExpression(*arg.getOperation());
        } else {
            // parameter
            pushParameter(*arg.getParameter());
            return 1;
        }
    }

    virtual unsigned pushExpression(Node& op) {
        if (getVariableID(op) > 0) {
            // use variable name
            _streamStack << createVariableName(op);
            return 1;
        } else {
            // print expression code
            _streamStack << op;
            return 0;
        }
    }

    virtual unsigned pushExpressionNoVarCheck2(Node& node) {
        Node* n;

        unsigned lines = pushExpressionNoVarCheck(node);

        while (true) {

            _streamStack.flush();
            if (!_streamStack.empty()) {
                n = &_streamStack.startNewOperationNode();
            } else {
                n = nullptr;
            }

            if (n == nullptr)
                break;

            unsigned lines2 = pushExpressionNoVarCheck(*n);

            lines = std::max<unsigned>(lines, lines2);
        }

        return lines;
    }

    virtual unsigned pushExpressionNoVarCheck(Node& node) {
        CGOpCode op = node.getOperationType();
        switch (op) {
            case CGOpCode::ArrayCreation:
                pushArrayCreationOp(node);
                break;
            case CGOpCode::SparseArrayCreation:
                pushSparseArrayCreationOp(node);
                break;
            case CGOpCode::ArrayElement:
                pushArrayElementOp(node);
                break;
            case CGOpCode::Assign:
                return pushAssignOp(node);

            case CGOpCode::Abs:
            case CGOpCode::Acos:
            case CGOpCode::Asin:
            case CGOpCode::Atan:
            case CGOpCode::Cosh:
            case CGOpCode::Cos:
            case CGOpCode::Exp:
            case CGOpCode::Log:
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
                pushUnaryFunction(node);
                break;
            case CGOpCode::AtomicForward: // atomicFunction.forward(q, p, vx, vy, tx, ty)
                pushAtomicForwardOp(node);
                break;
            case CGOpCode::AtomicReverse: // atomicFunction.reverse(p, tx, ty, px, py)
                pushAtomicReverseOp(node);
                break;
            case CGOpCode::Add:
                pushOperationAdd(node);
                break;
            case CGOpCode::Alias:
                return pushOperationAlias(node);

            case CGOpCode::ComLt:
            case CGOpCode::ComLe:
            case CGOpCode::ComEq:
            case CGOpCode::ComGe:
            case CGOpCode::ComGt:
            case CGOpCode::ComNe:
                pushConditionalAssignment(node);
                break;
            case CGOpCode::Div:
                pushOperationDiv(node);
                break;
            case CGOpCode::Inv:
                pushIndependentVariableName(node);
                break;
            case CGOpCode::Mul:
                pushOperationMul(node);
                break;
            case CGOpCode::Pow:
                pushPowFunction(node);
                break;
            case CGOpCode::Pri:
                pushPrintOperation(node);
                break;
            case CGOpCode::Sign:
                pushSignFunction(node);
                break;
            case CGOpCode::Sub:
                pushOperationMinus(node);
                break;

            case CGOpCode::UnMinus:
                pushOperationUnaryMinus(node);
                break;

            case CGOpCode::DependentMultiAssign:
                return pushDependentMultiAssign(node);

            case CGOpCode::Index:
                return 0; // nothing to do
            case CGOpCode::IndexAssign:
                pushIndexAssign(node);
                break;
            case CGOpCode::IndexDeclaration:
                return 0; // already done

            case CGOpCode::LoopStart:
                pushLoopStart(node);
                break;
            case CGOpCode::LoopIndexedIndep:
                pushLoopIndexedIndep(node);
                break;
            case CGOpCode::LoopIndexedDep:
                pushLoopIndexedDep(node);
                break;
            case CGOpCode::LoopIndexedTmp:
                pushLoopIndexedTmp(node);
                break;
            case CGOpCode::TmpDcl:
                // nothing to do
                return 0;
            case CGOpCode::Tmp:
                pushTmpVar(node);
                break;
            case CGOpCode::LoopEnd:
                pushLoopEnd(node);
                break;
            case CGOpCode::IndexCondExpr:
                pushIndexCondExprOp(node);
                break;
            case CGOpCode::StartIf:
                pushStartIf(node);
                break;
            case CGOpCode::ElseIf:
                pushElseIf(node);
                break;
            case CGOpCode::Else:
                pushElse(node);
                break;
            case CGOpCode::EndIf:
                pushEndIf(node);
                break;
            case CGOpCode::CondResult:
                pushCondResult(node);
                break;
            case CGOpCode::UserCustom:
                pushUserCustom(node);
                break;
            default:
                throw CGException("Unknown operation code '", op, "'.");
        }
        return 1;
    }

    virtual unsigned pushAssignOp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for assign operation")

        return push(node.getArguments()[0]);
    }

    virtual void pushUnaryFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary function")

        switch (op.getOperationType()) {
            case CGOpCode::Abs:
                _streamStack << absFuncName();
                break;
            case CGOpCode::Acos:
                _streamStack << acosFuncName();
                break;
            case CGOpCode::Asin:
                _streamStack << asinFuncName();
                break;
            case CGOpCode::Atan:
                _streamStack << atanFuncName();
                break;
            case CGOpCode::Cosh:
                _streamStack << coshFuncName();
                break;
            case CGOpCode::Cos:
                _streamStack << cosFuncName();
                break;
            case CGOpCode::Exp:
                _streamStack << expFuncName();
                break;
            case CGOpCode::Log:
                _streamStack << logFuncName();
                break;
            case CGOpCode::Sinh:
                _streamStack << sinhFuncName();
                break;
            case CGOpCode::Sin:
                _streamStack << sinFuncName();
                break;
            case CGOpCode::Sqrt:
                _streamStack << sqrtFuncName();
                break;
            case CGOpCode::Tanh:
                _streamStack << tanhFuncName();
                break;
            case CGOpCode::Tan:
                _streamStack << tanFuncName();
                break;
#if CPPAD_USE_CPLUSPLUS_2011
            case CGOpCode::Erf:
                _streamStack << erfFuncName();
                break;
            case CGOpCode::Erfc:
                _streamStack << erfcFuncName();
                break;
            case CGOpCode::Asinh:
                _streamStack << asinhFuncName();
                break;
            case CGOpCode::Acosh:
                _streamStack << acoshFuncName();
                break;
            case CGOpCode::Atanh:
                _streamStack << atanhFuncName();
                break;
            case CGOpCode::Expm1:
                _streamStack << expm1FuncName();
                break;
            case CGOpCode::Log1p:
                _streamStack << log1pFuncName();
                break;
#endif
            default:
                throw CGException("Unknown function name for operation code '", op.getOperationType(), "'.");
        }

        _streamStack << "(";
        push(op.getArguments()[0]);
        _streamStack << ")";
    }

    virtual void pushPowFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for pow() function")

        _streamStack <<powFuncName() << "(";
        push(op.getArguments()[0]);
        _streamStack << ", ";
        push(op.getArguments()[1]);
        _streamStack << ")";
    }

    virtual void pushSignFunction(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for sign() function")
        CPPADCG_ASSERT_UNKNOWN(op.getArguments()[0].getOperation() != nullptr)
        CPPADCG_ASSERT_UNKNOWN(getVariableID(*op.getArguments()[0].getOperation()) > 0)

        Node& arg = *op.getArguments()[0].getOperation();

        const std::string& argName = createVariableName(arg);

        _streamStack << "(" << argName << " " << _C_COMP_OP_GT << " ";
        pushParameter(Base(0.0));
        _streamStack << "?";
        pushParameter(Base(1.0));
        _streamStack << ":(" << argName << " " << _C_COMP_OP_LT << " ";
        pushParameter(Base(0.0));
        _streamStack << "?";
        pushParameter(Base(-1.0));
        _streamStack << ":";
        pushParameter(Base(0.0));
        _streamStack << "))";
    }

    virtual unsigned pushOperationAlias(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for alias")
        return push(op.getArguments()[0]);
    }

    virtual void pushOperationAdd(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for addition")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        if(right.getParameter() == nullptr || (*right.getParameter() >= 0)) {
            push(left);
            _streamStack << " + ";
            push(right);
        } else {
            // right has a negative parameter so we would get v0 + -v1
            push(left);
            _streamStack << " - ";
            pushParameter(-*right.getParameter()); // make it positive
        }
    }

    virtual void pushOperationMinus(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for subtraction")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        if(right.getParameter() == nullptr || (*right.getParameter() >= 0)) {
            bool encloseRight = encloseInParenthesesMul(right.getOperation());

            push(left);
            _streamStack << " - ";
            if (encloseRight) {
                _streamStack << "(";
            }
            push(right);
            if (encloseRight) {
                _streamStack << ")";
            }
        } else {
            // right has a negative parameter so we would get v0 - -v1
            push(left);
            _streamStack << " + ";
            pushParameter(-*right.getParameter()); // make it positive
        }
    }

    inline bool encloseInParenthesesDiv(const Node* node) const {
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
    }

    virtual void pushOperationDiv(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for division")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        bool encloseLeft = encloseInParenthesesDiv(left.getOperation());
        bool encloseRight = encloseInParenthesesDiv(right.getOperation());

        if (encloseLeft) {
            _streamStack << "(";
        }
        push(left);
        if (encloseLeft) {
            _streamStack << ")";
        }
        _streamStack << " / ";
        if (encloseRight) {
            _streamStack << "(";
        }
        push(right);
        if (encloseRight) {
            _streamStack << ")";
        }
    }

    inline bool encloseInParenthesesMul(const Node* node) const {
        while (node != nullptr) {
            if (getVariableID(*node) != 0)
                return false;
            else if (node->getOperationType() == CGOpCode::Alias)
                node = node->getArguments()[0].getOperation();
            else
                break;
        }
        return node != nullptr &&
                getVariableID(*node) == 0 &&
                node->getOperationType() != CGOpCode::Div &&
                node->getOperationType() != CGOpCode::Mul &&
                !isFunction(node->getOperationType());
    }

    virtual void pushOperationMul(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 2, "Invalid number of arguments for multiplication")

        const Arg& left = op.getArguments()[0];
        const Arg& right = op.getArguments()[1];

        bool encloseLeft = encloseInParenthesesMul(left.getOperation());
        bool encloseRight = encloseInParenthesesMul(right.getOperation());

        if (encloseLeft) {
            _streamStack << "(";
        }
        push(left);
        if (encloseLeft) {
            _streamStack << ")";
        }
        _streamStack << " * ";
        if (encloseRight) {
            _streamStack << "(";
        }
        push(right);
        if (encloseRight) {
            _streamStack << ")";
        }
    }

    virtual void pushOperationUnaryMinus(Node& op) {
        CPPADCG_ASSERT_KNOWN(op.getArguments().size() == 1, "Invalid number of arguments for unary minus")

        const Arg& arg = op.getArguments()[0];

        bool enclose = encloseInParenthesesMul(arg.getOperation());

        _streamStack << "-";
        if (enclose) {
            _streamStack << "(";
        } else {
            _streamStack << " "; // there may be several - together -> space required
        }
        push(arg);
        if (enclose) {
            _streamStack << ")";
        }
    }

    virtual void pushPrintOperation(const Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Pri, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for print operation")

        const auto& pnode = static_cast<const PrintOperationNode<Base>&> (node);
        std::string before = pnode.getBeforeString();
        replaceString(before, "\n", "\\n");
        replaceString(before, "\"", "\\\"");
        std::string after = pnode.getAfterString();
        replaceString(after, "\n", "\\n");
        replaceString(after, "\"", "\\\"");

        _streamStack <<_indentation << "fprintf(stderr, \"" << before << getPrintfBaseFormat() << after << "\"";
        const std::vector<Arg>& args = pnode.getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            _streamStack << ", ";
            push(args[a]);
        }
        _streamStack << ");\n";
    }

    virtual void pushConditionalAssignment(Node& node) {
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
            pushAssignmentStart(node, varName, isDep);
            push(trueCase);
            pushAssignmentEnd(node);
        } else {
            _streamStack <<_indentation << "if( ";
            push(left);
            _streamStack << " " << getComparison(node.getOperationType()) << " ";
            push(right);
            _streamStack << " ) {\n";
            _streamStack <<_spaces;
            pushAssignmentStart(node, varName, isDep);
            push(trueCase);
            pushAssignmentEnd(node);
            _streamStack <<_indentation << "} else {\n";
            _streamStack <<_spaces;
            pushAssignmentStart(node, varName, isDep);
            push(falseCase);
            pushAssignmentEnd(node);
            _streamStack <<_indentation << "}\n";
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

    virtual void pushArrayCreationOp(Node& op);

    virtual void pushSparseArrayCreationOp(Node& op);

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

    virtual void pushArrayElementOp(Node& op);

    virtual void pushAtomicForwardOp(Node& atomicFor) {
        CPPADCG_ASSERT_KNOWN(atomicFor.getInfo().size() == 3, "Invalid number of information elements for atomic forward operation")
        int q = atomicFor.getInfo()[1];
        int p = atomicFor.getInfo()[2];
        size_t p1 = p + 1;
        const std::vector<Arg>& opArgs = atomicFor.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 2, "Invalid number of arguments for atomic forward operation")

        size_t id = atomicFor.getInfo()[0];
        size_t atomicIndex = _info->atomicFunctionId2Index.at(id);

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
            printArrayStructInit(_ATOMIC_TX, k, tx, k); // also does indentation
        }
        // ty
        printArrayStructInit(_ATOMIC_TY, *ty[p]); // also does indentation
        _ss.str("");

        _streamStack << _indentation << "atomicFun.forward(atomicFun.libModel, "
                     << atomicIndex << ", " << q << ", " << p << ", "
                     << _ATOMIC_TX << ", &" << _ATOMIC_TY << "); // "
                     << _info->atomicFunctionId2Name.at(id)
                     << "\n";

        /**
         * the values of ty are now changed
         */
        markArrayChanged(*ty[p]);
    }

    virtual void pushAtomicReverseOp(Node& atomicRev) {
        CPPADCG_ASSERT_KNOWN(atomicRev.getInfo().size() == 2, "Invalid number of information elements for atomic reverse operation")
        int p = atomicRev.getInfo()[1];
        size_t p1 = p + 1;
        const std::vector<Arg>& opArgs = atomicRev.getArguments();
        CPPADCG_ASSERT_KNOWN(opArgs.size() == p1 * 4, "Invalid number of arguments for atomic reverse operation")

        size_t id = atomicRev.getInfo()[0];
        size_t atomicIndex = _info->atomicFunctionId2Index.at(id);
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
            printArrayStructInit(_ATOMIC_TX, k, tx, k); // also does indentation
        }
        // py
        for (size_t k = 0; k < p1; k++) {
            printArrayStructInit(_ATOMIC_PY, k, py, k); // also does indentation
        }
        // px
        printArrayStructInit(_ATOMIC_PX, *px[0]); // also does indentation
        _ss.str("");

        _streamStack << _indentation << "atomicFun.reverse(atomicFun.libModel, "
                     << atomicIndex << ", " << p << ", "
                     << _ATOMIC_TX << ", &" << _ATOMIC_PX << ", " << _ATOMIC_PY << "); // "
                     << _info->atomicFunctionId2Name.at(id)
                     << "\n";

        /**
         * the values of px are now changed
         */
        markArrayChanged(*px[0]);
    }

    virtual unsigned pushDependentMultiAssign(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::DependentMultiAssign, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments")

        const std::vector<Arg>& args = node.getArguments();
        for (size_t a = 0; a < args.size(); a++) {
            bool useArg;
            const Arg& arg = args[a];
            if (arg.getParameter() != nullptr) {
                useArg = true;
            } else {
                CGOpCode op = arg.getOperation()->getOperationType();
                useArg = op != CGOpCode::DependentRefRhs && op != CGOpCode::LoopEnd && op != CGOpCode::EndIf;
            }

            if (useArg) {
                pushAssignment(node, arg); // ignore other arguments!
                return 1;
            }
        }
        return 0;
    }

    virtual void pushLoopStart(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopStart, "Invalid node type")

        auto& lnode = static_cast<LoopStartOperationNode<Base>&> (node);
        _currentLoops.push_back(&lnode);

        const std::string& jj = *lnode.getIndex().getName();
        std::string iterationCount;
        if (lnode.getIterationCountNode() != nullptr) {
            iterationCount = *lnode.getIterationCountNode()->getIndex().getName();
        } else {
            std::ostringstream oss;
            oss << lnode.getIterationCount();
            iterationCount = oss.str();
        }

        _streamStack << _spaces << "for("
                     << jj << " = 0; "
                     << jj << " < " << iterationCount << "; "
                     << jj << "++) {\n";
        _indentation += _spaces;
    }

    virtual void pushLoopEnd(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopEnd, "Invalid node type")

        _indentation.resize(_indentation.size() - _spaces.size());

        _streamStack <<_indentation << "}\n";

        _currentLoops.pop_back();
    }


    virtual size_t printLoopIndexDeps(const std::vector<Node*>& variableOrder,
                                      size_t pos);

    virtual size_t printLoopIndexedDepsUsingLoop(const std::vector<Node*>& variableOrder,
                                                 size_t starti);

    virtual void pushLoopIndexedDep(Node& node);

    virtual void pushLoopIndexedIndep(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedIndep, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getInfo().size() == 1, "Invalid number of information elements for loop indexed independent operation")

        // CGLoopIndexedIndepOp
        size_t pos = node.getInfo()[1];
        const IndexPattern* ip = _info->loopIndependentIndexPatterns[pos];
        _streamStack <<_nameGen->generateIndexedIndependent(node, getVariableID(node), *ip);
    }

    virtual void pushLoopIndexedTmp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::LoopIndexedTmp, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for loop indexed temporary operation")
        Node* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")

        push(node.getArguments()[1]);
    }

    virtual void pushTmpVar(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Tmp, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for temporary variable usage operation")
        Node* tmpVar = node.getArguments()[0].getOperation();
        CPPADCG_ASSERT_KNOWN(tmpVar != nullptr && tmpVar->getOperationType() == CGOpCode::TmpDcl, "Invalid arguments for loop indexed temporary operation")

        _streamStack <<*tmpVar->getName();
    }

    virtual void pushIndexAssign(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexAssign, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() > 0, "Invalid number of arguments for an index assignment operation")

        auto& inode = static_cast<IndexAssignOperationNode<Base>&> (node);

        const IndexPattern& ip = inode.getIndexPattern();
        _streamStack <<_indentation << (*inode.getIndex().getName())
                << " = " << indexPattern2String(ip, inode.getIndexPatternIndexes()) << ";\n";
    }

    virtual void pushIndexCondExprOp(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::IndexCondExpr, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 1, "Invalid number of arguments for an index condition expression operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an index condition expression operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation()->getOperationType() == CGOpCode::Index, "Invalid argument for an index condition expression operation")

        const std::vector<size_t>& info = node.getInfo();

        auto& iterationIndexOp = static_cast<IndexOperationNode<Base>&> (*node.getArguments()[0].getOperation());
        const std::string& index = *iterationIndexOp.getIndex().getName();

        printIndexCondExpr(_code, info, index);
    }

    virtual void pushStartIf(Node& node) {
        /**
         * the first argument is the condition, following arguments are
         * just extra dependencies that must be defined outside the if
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::StartIf, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'if start' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'if start' operation")

        _streamStack <<_indentation << "if(";
        pushIndexCondExprOp(*node.getArguments()[0].getOperation());
        _streamStack << ") {\n";

        _indentation += _spaces;
    }

    virtual void pushElseIf(Node& node) {
        /**
         * the first argument is the condition, the second argument is the
         * if start node, the following arguments are assignments in the
         * previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::ElseIf, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 2, "Invalid number of arguments for an 'else if' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an 'else if' operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an 'else if' operation")

        _indentation.resize(_indentation.size() - _spaces.size());

        _streamStack <<_indentation << "} else if(";
        pushIndexCondExprOp(*node.getArguments()[1].getOperation());
        _streamStack << ") {\n";

        _indentation += _spaces;
    }

    virtual void pushElse(Node& node) {
        /**
         * the first argument is the  if start node, the following arguments
         * are assignments in the previous if branch
         */
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::Else, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() >= 1, "Invalid number of arguments for an 'else' operation")

        _indentation.resize(_indentation.size() - _spaces.size());

        _streamStack <<_indentation << "} else {\n";

        _indentation += _spaces;
    }

    virtual void pushEndIf(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::EndIf, "Invalid node type for an 'end if' operation")

        _indentation.resize(_indentation.size() - _spaces.size());

        _streamStack <<_indentation << "}\n";
    }

    virtual void pushCondResult(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::CondResult, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(node.getArguments().size() == 2, "Invalid number of arguments for an assignment inside an if/else operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[0].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation")
        CPPADCG_ASSERT_KNOWN(node.getArguments()[1].getOperation() != nullptr, "Invalid argument for an an assignment inside an if/else operation")

        // just follow the argument
        Node& nodeArg = *node.getArguments()[1].getOperation();
        printAssignment(nodeArg);
    }

    virtual void pushUserCustom(Node& node) {
        CPPADCG_ASSERT_KNOWN(node.getOperationType() == CGOpCode::UserCustom, "Invalid node type")

        throw CGException("Unable to generate C source code for user custom operation nodes.");
    }

    inline bool isDependent(const Node& arg) const {
        if (arg.getOperationType() == CGOpCode::LoopIndexedDep) {
            return true;
        }
        size_t id = getVariableID(arg);
        return id > _independentSize && id < _minTemporaryVarID;
    }

    virtual void printParameter(const Base& value) {
        writeParameter(value, _code);
    }

    virtual void pushParameter(const Base& value) {
        writeParameter(value, _streamStack);
    }

    template<class Output>
    void writeParameter(const Base& value, Output& output) {
        // make sure all digits of floating point values are printed
        std::ostringstream os;
        os << std::setprecision(_parameterPrecision) << value;

        std::string number = os.str();
        output << number;

        if (std::abs(value) > Base(0) && value != Base(1) && value != Base(-1)) {
            if (number.find('.') == std::string::npos && number.find('e') == std::string::npos) {
                // also make sure there is always a '.' after the number in
                // order to avoid integer overflows
                output << '.';
            }
        }
    }

    virtual const std::string& getComparison(enum CGOpCode op) const {
        switch (op) {
            case CGOpCode::ComLt:
                return _C_COMP_OP_LT;

            case CGOpCode::ComLe:
                return _C_COMP_OP_LE;

            case CGOpCode::ComEq:
                return _C_COMP_OP_EQ;

            case CGOpCode::ComGe:
                return _C_COMP_OP_GE;

            case CGOpCode::ComGt:
                return _C_COMP_OP_GT;

            case CGOpCode::ComNe:
                return _C_COMP_OP_NE;

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

    inline static bool isCondAssign(enum CGOpCode op) {
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
private:

    class AtomicFuncArray {
    public:
        std::string data;
        unsigned long size;
        bool sparse;
        size_t idx_id;
        unsigned long nnz;
        unsigned short scope;
    };
};
template<class Base>
const std::string LanguageC<Base>::U_INDEX_TYPE = "unsigned long"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_LT = "<"; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_LE = "<="; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_EQ = "=="; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_GE = ">="; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_GT = ">"; // NOLINT(cert-err58-cpp)
template<class Base>
const std::string LanguageC<Base>::_C_COMP_OP_NE = "!="; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_C_STATIC_INDEX_ARRAY = "index"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_C_SPARSE_INDEX_ARRAY = "idx"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_TX = "atx"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_TY = "aty"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_PX = "apx"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::_ATOMIC_PY = "apy"; // NOLINT(cert-err58-cpp)

template<class Base>
const std::string LanguageC<Base>::ATOMICFUN_STRUCT_DEFINITION = // NOLINT(cert-err58-cpp)
"typedef struct Array {\n"
"    void* data;\n"
"    " + U_INDEX_TYPE + " size;\n"
"    int sparse;\n"
"    const " + U_INDEX_TYPE + "* idx;\n"
"    " + U_INDEX_TYPE + " nnz;\n"
"} Array;\n"
"\n"
"struct LangCAtomicFun {\n"
"    void* libModel;\n"
"    int (*forward)(void* libModel,\n"
"                   int atomicIndex,\n"
"                   int q,\n"
"                   int p,\n"
"                   const Array tx[],\n"
"                   Array* ty);\n"
"    int (*reverse)(void* libModel,\n"
"                   int atomicIndex,\n"
"                   int p,\n"
"                   const Array tx[],\n"
"                   Array* px,\n"
"                   const Array py[]);\n"
"};";

} // END cg namespace
} // END CppAD namespace

#endif
