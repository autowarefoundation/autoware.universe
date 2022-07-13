#ifndef CPPAD_CG_LANG_C_DEFAULT_HESSIAN_VAR_NAME_GEN_INCLUDED
#define CPPAD_CG_LANG_C_DEFAULT_HESSIAN_VAR_NAME_GEN_INCLUDED
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
 * Creates variables names for the source code generated for Hessian
 * calculations.
 * The independent variables are considered to have been registered first as
 * variable in the code generation handler and then the multipliers.
 *
 * @author Joao Leal
 */
template<class Base>
class LangCDefaultHessianVarNameGenerator : public VariableNameGenerator<Base> {
protected:
    VariableNameGenerator<Base>* _nameGen;
    // the lowest variable ID used for the equation multipliers
    const size_t _minMultiplierID;
    // array name of the independent variables
    const std::string _multName;
    // auxiliary string stream
    std::stringstream _ss;
public:

    LangCDefaultHessianVarNameGenerator(VariableNameGenerator<Base>* nameGen,
                                        size_t n) :
        _nameGen(nameGen),
        _minMultiplierID(n + 1),
        _multName("mult") {

        CPPADCG_ASSERT_KNOWN(_nameGen != nullptr, "The name generator must not be NULL")

        initialize();
    }

    LangCDefaultHessianVarNameGenerator(VariableNameGenerator<Base>* nameGen,
                                        std::string multName,
                                        size_t n) :
        _nameGen(nameGen),
        _minMultiplierID(n + 1),
        _multName(std::move(multName)) {

        CPPADCG_ASSERT_KNOWN(_nameGen != nullptr, "The name generator must not be null")
        CPPADCG_ASSERT_KNOWN(_multName.size() > 0, "The name for the multipliers must not be empty")

        initialize();
    }

    inline virtual ~LangCDefaultHessianVarNameGenerator() = default;

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
        if (id < _minMultiplierID) {
            return _nameGen->generateIndependent(independent, id);
        }

        _ss.clear();
        _ss.str("");
        _ss << _multName << "[" << (id - _minMultiplierID) << "]";
        return _ss.str();
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

    std::string generateIndexedIndependent(const OperationNode<Base>& indexedIndep,
                                           size_t id,
                                           const IndexPattern& ip) override {
        bool isX = indexedIndep.getInfo()[0] == 0;
        if (isX) {
            return _nameGen->generateIndexedIndependent(indexedIndep, id, ip);
        }

        size_t nIndex = indexedIndep.getArguments().size();

        CPPADCG_ASSERT_KNOWN(indexedIndep.getOperationType() == CGOpCode::LoopIndexedIndep, "Invalid node type")
        CPPADCG_ASSERT_KNOWN(nIndex > 0, "Invalid number of arguments")

        std::vector<const OperationNode<Base>*> indices(nIndex);
        for (size_t i = 0; i < nIndex; ++i) {// typically there is only one index but there may be more
            CPPADCG_ASSERT_KNOWN(indexedIndep.getArguments()[i].getOperation() != nullptr, "Invalid argument")
            CPPADCG_ASSERT_KNOWN(indexedIndep.getArguments()[i].getOperation()->getOperationType() == CGOpCode::Index, "Invalid argument")
            indices[i] = &static_cast<const IndexOperationNode<Base>&> (*indexedIndep.getArguments()[i].getOperation()).getIndex();
        }

        _ss.clear();
        _ss.str("");

        _ss << _multName << "[" << LanguageC<Base>::indexPattern2String(ip, indices) << "]";
        return _ss.str();
    }

    const std::string& getIndependentArrayName(const OperationNode<Base>& indep,
                                               size_t id) override {
        if (id < _minMultiplierID)
            return _nameGen->getIndependentArrayName(indep, id);
        else
            return _multName;
    }

    size_t getIndependentArrayIndex(const OperationNode<Base>& indep,
                                    size_t id) override {
        if (id < _minMultiplierID)
            return _nameGen->getIndependentArrayIndex(indep, id);
        else
            return id - _minMultiplierID;
    }

    bool isConsecutiveInIndepArray(const OperationNode<Base>& indepFirst,
                                   size_t id1,
                                   const OperationNode<Base>& indepSecond,
                                   size_t id2) override {
        if ((id1 < _minMultiplierID) != (id2 < _minMultiplierID))
            return false;

        if (id1 < _minMultiplierID && id2 < _minMultiplierID)
            return _nameGen->isConsecutiveInIndepArray(indepFirst, id1, indepSecond, id2);
        else
            return id1 + 1 == id2;
    }

    bool isInSameIndependentArray(const OperationNode<Base>& indep1,
                                  size_t id1,
                                  const OperationNode<Base>& indep2,
                                  size_t id2) override {
        size_t l1;
        if (indep1.getOperationType() == CGOpCode::Inv) {
            l1 = id1 < _minMultiplierID ? 0 : 1;
        } else {
            l1 = indep1.getInfo()[0]; //CGLoopIndexedIndepOp
        }

        size_t l2;
        if (indep2.getOperationType() == CGOpCode::Inv) {
            l2 = id2 < _minMultiplierID ? 0 : 1;
        } else {
            l2 = indep2.getInfo()[0]; //CGLoopIndexedIndepOp
        }

        return l1 == l2;
    }

    void setTemporaryVariableID(size_t minTempID,
                                size_t maxTempID,
                                size_t maxTempArrayID,
                                size_t maxTempSparseArrayID) override {
        _nameGen->setTemporaryVariableID(minTempID, maxTempID, maxTempArrayID, maxTempSparseArrayID);
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

private:

    inline void initialize() {
        this->_independent = _nameGen->getIndependent(); // copy

        this->_independent.push_back(FuncArgument(_multName));
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
