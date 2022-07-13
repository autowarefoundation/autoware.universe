#ifndef CPPAD_CG_LANG_C_DEFAULT_REVERSE2_VAR_NAME_GEN_INCLUDED
#define CPPAD_CG_LANG_C_DEFAULT_REVERSE2_VAR_NAME_GEN_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * Creates variables names for the source code generated for second-order
 * reverse mode calculations.
 * The independent variables are considered to have been registered first,
 * followed by a first level of additional variables and then a second.
 *
 * @author Joao Leal
 */
template<class Base>
class LangCDefaultReverse2VarNameGenerator : public VariableNameGenerator<Base> {
protected:
    VariableNameGenerator<Base>* _nameGen;
    // the lowest variable ID used for the first independent variable level
    const size_t _minLevel1ID;
    // array name of the independent variables (1st level)
    const std::string _level1Name;
    // the lowest variable ID used for the second independent variable level
    const size_t _minLevel2ID;
    // array name of the independent variables (2nd level)
    const std::string _level2Name;
    // auxiliary string stream
    std::stringstream _ss;
public:

    LangCDefaultReverse2VarNameGenerator(VariableNameGenerator<Base>* nameGen,
                                         size_t n,
                                         size_t n1) :
        _nameGen(nameGen),
        _minLevel1ID(n + 1),
        _level1Name("tx1"),
        _minLevel2ID(_minLevel1ID + n1),
        _level2Name("py2") {

        CPPADCG_ASSERT_KNOWN(_nameGen != nullptr, "The name generator must not be null")

        initialize();
    }

    LangCDefaultReverse2VarNameGenerator(VariableNameGenerator<Base>* nameGen,
                                         size_t n,
                                         std::string level1Name,
                                         size_t n1,
                                         std::string level2Name) :
        _nameGen(nameGen),
        _minLevel1ID(n + 1),
        _level1Name(std::move(level1Name)),
        _minLevel2ID(_minLevel1ID + n1),
        _level2Name(std::move(level2Name)) {

        CPPADCG_ASSERT_KNOWN(_nameGen != nullptr, "The name generator must not be null")
        CPPADCG_ASSERT_KNOWN(_level1Name.size() > 0, "The name for the first level must not be empty")
        CPPADCG_ASSERT_KNOWN(_level2Name.size() > 0, "The name for the second level must not be empty")

        initialize();
    }

    inline virtual ~LangCDefaultReverse2VarNameGenerator() = default;

    const std::vector<FuncArgument>& getDependent() const override {
        return _nameGen->getDependent();
    }

    const std::vector<FuncArgument>& getTemporary() const override {
        return _nameGen->getTemporary();
    }

    size_t getMinTemporaryVariableID() const override {
        return _nameGen->getMinTemporaryVariableID();
    }

    size_t getMaxTemporaryVariableID() const override {
        return _nameGen->getMaxTemporaryVariableID();
    }

    size_t getMaxTemporaryArrayVariableID() const override {
        return _nameGen->getMaxTemporaryArrayVariableID();
    }

    size_t getMaxTemporarySparseArrayVariableID() const override {
        return _nameGen->getMaxTemporarySparseArrayVariableID();
    }

    std::string generateDependent(size_t index) override {
        return _nameGen->generateDependent(index);
    }

    std::string generateIndependent(const OperationNode<Base>& independent,
                                    size_t id) override {
        if (id < _minLevel1ID) {
            return _nameGen->generateIndependent(independent, id);
        } else {
            _ss.clear();
            _ss.str("");
            if (id < _minLevel2ID) {
                _ss << _level1Name << "[" << (id - _minLevel1ID) << "]";
            } else {
                _ss << _level2Name << "[" << (id - _minLevel2ID) << "]";
            }
            return _ss.str();
        }
    }

    std::string generateTemporary(const OperationNode<Base>& variable,
                                  size_t id) override {
        return _nameGen->generateTemporary(variable, id);
    }

    std::string generateTemporaryArray(const OperationNode<Base>& variable,
                                       size_t id) override {
        return _nameGen->generateTemporaryArray(variable, id);
    }

    std::string generateTemporarySparseArray(const OperationNode<Base>& variable,
                                             size_t id) override {
        return _nameGen->generateTemporarySparseArray(variable, id);
    }

    std::string generateIndexedDependent(const OperationNode<Base>& var,
                                         size_t id,
                                         const IndexPattern& ip) override {
        return _nameGen->generateIndexedDependent(var, id, ip);
    }

    std::string generateIndexedIndependent(const OperationNode<Base>& independent,
                                           size_t id,
                                           const IndexPattern& ip) override {
        size_t varType = independent.getInfo()[0];
        if (varType == 0) {
            return _nameGen->generateIndexedIndependent(independent, id, ip);
        } else {
            size_t nIndex = independent.getArguments().size();

            CPPADCG_ASSERT_KNOWN(independent.getOperationType() == CGOpCode::LoopIndexedIndep, "Invalid node type")
            CPPADCG_ASSERT_KNOWN(nIndex > 0, "Invalid number of arguments")

            _ss.clear();
            _ss.str("");

            std::vector<const OperationNode<Base>*> indices(nIndex);
            for (size_t i = 0; i < nIndex; ++i) {// typically there is only one index but there may be more
                CPPADCG_ASSERT_KNOWN(independent.getArguments()[i].getOperation() != nullptr, "Invalid argument")
                CPPADCG_ASSERT_KNOWN(independent.getArguments()[i].getOperation()->getOperationType() == CGOpCode::Index, "Invalid argument")
                indices[i] = &static_cast<const IndexOperationNode<Base>&> (*independent.getArguments()[i].getOperation()).getIndex();
            }

            if (varType == 1) {
                _ss << _level1Name << "[" << LanguageC<Base>::indexPattern2String(ip, indices) << "]";
            } else {
                _ss << _level2Name << "[" << LanguageC<Base>::indexPattern2String(ip, indices) << "]";
            }
            return _ss.str();
        }

    }

    const std::string& getIndependentArrayName(const OperationNode<Base>& indep,
                                               size_t id) override {
        if (id < _minLevel1ID)
            return _nameGen->getIndependentArrayName(indep, id);
        else if (id < _minLevel2ID)
            return _level1Name;
        else
            return _level2Name;
    }

    size_t getIndependentArrayIndex(const OperationNode<Base>& indep,
                                    size_t id) override {
        if (id < _minLevel1ID)
            return _nameGen->getIndependentArrayIndex(indep, id);
        else if (id < _minLevel2ID)
            return id - _minLevel1ID;
        else
            return id - _minLevel2ID;
    }

    bool isConsecutiveInIndepArray(const OperationNode<Base>& indepFirst,
                                   size_t id1,
                                   const OperationNode<Base>& indepSecond,
                                   size_t id2) override {
        if ((id1 < _minLevel1ID) != (id2 < _minLevel1ID))
            return false;

        if (id1 < _minLevel1ID && id2 < _minLevel1ID)
            return _nameGen->isConsecutiveInIndepArray(indepFirst, id1, indepSecond, id2);

        if ((id1 < _minLevel2ID) != (id2 < _minLevel2ID))
            return false;

        return id1 + 1 == id2;
    }

    bool isInSameIndependentArray(const OperationNode<Base>& indep1,
                                  size_t id1,
                                  const OperationNode<Base>& indep2,
                                  size_t id2) override {
        size_t l1;
        if (indep1.getOperationType() == CGOpCode::Inv) {
            l1 = id1 < _minLevel1ID ? 0 : (id1 < _minLevel2ID ? 1 : 2);
        } else {
            l1 = indep1.getInfo()[0]; //CGLoopIndexedIndepOp
        }

        size_t l2;
        if (indep2.getOperationType() == CGOpCode::Inv) {
            l2 = id2 < _minLevel1ID ? 0 : (id2 < _minLevel2ID ? 1 : 2);
        } else {
            l2 = indep2.getInfo()[0]; //CGLoopIndexedIndepOp
        }

        return l1 == l2;
    }

    const std::string& getTemporaryVarArrayName(const OperationNode<Base>& var,
                                                size_t id) override {
        return _nameGen->getTemporaryVarArrayName(var, id);
    }

    size_t getTemporaryVarArrayIndex(const OperationNode<Base>& var,
                                     size_t id) override {
        return _nameGen->getTemporaryVarArrayIndex(var, id);
    }

    bool isConsecutiveInTemporaryVarArray(const OperationNode<Base>& varFirst,
                                          size_t idFirst,
                                          const OperationNode<Base>& varSecond,
                                          size_t idSecond) override {
        return _nameGen->isConsecutiveInTemporaryVarArray(varFirst, idFirst, varSecond, idSecond);
    }

    bool isInSameTemporaryVarArray(const OperationNode<Base>& var1,
                                   size_t id1,
                                   const OperationNode<Base>& var2,
                                   size_t id2) override {
        return _nameGen->isInSameTemporaryVarArray(var1, id1, var2, id2);
    }

    void setTemporaryVariableID(size_t minTempID,
                                size_t maxTempID,
                                size_t maxTempArrayID,
                                size_t maxTempSparseArrayID) override {
        _nameGen->setTemporaryVariableID(minTempID, maxTempID, maxTempArrayID, maxTempSparseArrayID);
    }

private:

    inline void initialize() {
        this->_independent = _nameGen->getIndependent(); // copy
        this->_independent.push_back(FuncArgument(_level1Name));
        this->_independent.push_back(FuncArgument(_level2Name));
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
