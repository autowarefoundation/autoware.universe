#ifndef CPPAD_CG_VARIABLE_NAME_GENERATOR_INCLUDED
#define CPPAD_CG_VARIABLE_NAME_GENERATOR_INCLUDED
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
 * Function arguments
 */
typedef struct FuncArgument {
    std::string name;
    bool array;

    inline FuncArgument() :
        array(false) {
    }

    inline FuncArgument(const std::string& nam, bool a = true) :
        name(nam),
        array(a) {
    }
} FuncArgument;

/**
 * Creates variables names for the source code.
 * 
 * @author Joao Leal
 */
template<class Base>
class VariableNameGenerator {
protected:
    std::vector<FuncArgument> _dependent;
    std::vector<FuncArgument> _independent;
    std::vector<FuncArgument> _temporary;
public:

    /**
     * Provides the dependent variable arguments used by a function.
     * 
     * @return the dependent variable arguments
     */
    virtual const std::vector<FuncArgument>& getDependent() const {
        return _dependent;
    }

    /**
     * Provides the independent variable arguments used by a function.
     * 
     * @return the independent variable arguments
     */
    virtual const std::vector<FuncArgument>& getIndependent() const {
        return _independent;
    }

    /**
     * Provides the temporary variable arguments used by a function.
     * 
     * @return the temporary variable arguments
     */
    virtual const std::vector<FuncArgument>& getTemporary() const {
        return _temporary;
    }

    /**
     * Provides the minimum variable ID of temporary variables.
     */
    virtual size_t getMinTemporaryVariableID() const = 0;

    /**
     * Provides the maximum used variable ID of temporary variables.
     */
    virtual size_t getMaxTemporaryVariableID() const = 0;

    /**
     * Provides the maximum variable ID of temporary dense array variables.
     */
    virtual size_t getMaxTemporaryArrayVariableID() const = 0;

    /**
     * Provides the maximum variable ID of temporary sparse array variables.
     */
    virtual size_t getMaxTemporarySparseArrayVariableID() const = 0;

    /**
     * Creates a name for a dependent variable.
     * 
     * @param index the dependent variable index
     * @return the generated name
     */
    virtual std::string generateDependent(size_t index) = 0;

    /**
     * Creates a name for a dependent variable.
     * 
     * @param variable the node representing the independent variable
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (unique for independent variables)
     * @return the generated name
     */
    virtual std::string generateIndependent(const OperationNode<Base>& variable,
                                            size_t id) = 0;

    /**
     * Creates a name for a temporary variable.
     * 
     * @param variable the node representing the temporary variable
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (potentially not unique)
     * @return the generated name
     */
    virtual std::string generateTemporary(const OperationNode<Base>& variable,
                                          size_t id) = 0;

    /**
     * Creates a name for a temporary dense array variable.
     * 
     * @param variable the node representing the dense array variable creation
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (potentially not unique)
     * @return the generated name
     */
    virtual std::string generateTemporaryArray(const OperationNode<Base>& variable,
                                               size_t id) = 0;

    /**
     * Creates a name for a temporary sparse array variable.
     * 
     * @param variable the node representing the sparse array variable creation
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (potentially not unique)
     * @return the generated name
     */
    virtual std::string generateTemporarySparseArray(const OperationNode<Base>& variable,
                                                     size_t id) = 0;

    /**
     * Creates a name for a reference to an indexed dependent variable
     * expression.
     * 
     * @param var the node representing an indexed dependent variable
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (potentially not unique)
     * @param ip the index pattern
     * @return the generated name
     */
    virtual std::string generateIndexedDependent(const OperationNode<Base>& var,
                                                 size_t id,
                                                 const IndexPattern& ip) = 0;

    /**
     * Creates a name for a reference to an indexed independent variable 
     * expression.
     * 
     * @param var the node representing an indexed independent variable
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (unique for indexed independent variables)
     * @param ip the index pattern
     * @return the generated name
     */
    virtual std::string generateIndexedIndependent(const OperationNode<Base>& var,
                                                   size_t id,
                                                   const IndexPattern& ip) = 0;

    /**
     * Defines the ID ranges used by each variable type.
     * 
     * @param minTempID the lowest ID of temporary variables
     * @param maxTempID the highest used ID of temporary variables
     * @param maxTempArrayID the highest used ID of temporary dense array
     *                       variables
     * @param maxTempSparseArrayID the highest used ID of temporary sparse
     *                             array variables
     */
    virtual void setTemporaryVariableID(size_t minTempID,
                                        size_t maxTempID,
                                        size_t maxTempArrayID,
                                        size_t maxTempSparseArrayID) = 0;

    /**
     * Provides the array name where independent variables are provided to the 
     * function.
     * It should only be called if independents are saved in an array.
     * 
     * @param indep the independent variable node (CGInvOp)
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (unique for independent variable arrays)
     * @return the array name
     */
    virtual const std::string& getIndependentArrayName(const OperationNode<Base>& indep,
                                                       size_t id) = 0;

    /**
     * Provides the index in the associated independent array of an 
     * independent variable.
     * It should only be called if independents are saved in an array.
     * 
     * @param indep the independent variable node (CGInvOp)
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (unique for independent variable arrays)
     * @return the index
     */
    virtual size_t getIndependentArrayIndex(const OperationNode<Base>& indep,
                                            size_t id) = 0;

    /**
     * Whether or not two independent variables are considered to be part of
     * the same independent variable array at consecutive locations.
     * 
     * @param indepFirst the independent node (CGInvOp) with the lower index
     * @param idFirst an ID assigned by the CodeHandler to the first node
     *                (unique for independent variables)
     * @param indepSecond the independent node (CGInvOp) with the higher index
     * @param idSecond an ID assigned by the CodeHandler to the second node
     *                 (unique for independent variables)
     * @return true if the independents are consecutive
     */
    virtual bool isConsecutiveInIndepArray(const OperationNode<Base>& indepFirst,
                                           size_t idFirst,
                                           const OperationNode<Base>& indepSecond,
                                           size_t idSecond) = 0;

    /**
     * Determines whether or not two independents are part of the same
     * independent variable array.
     * 
     * @param indep1 the first independent node (CGInvOp or CGLoopIndexedIndepOp)
     * @param id1 an ID assigned by the CodeHandler to indep1
     *            (unique for independent variables)
     * @param indep2 the second independent node (CGInvOp or CGLoopIndexedIndepOp)
     * @param id2 an ID assigned by the CodeHandler to indep2
     *            (unique for independent variables)
     * @return true if the independents are part of the same array
     */
    virtual bool isInSameIndependentArray(const OperationNode<Base>& indep1,
                                          size_t id1,
                                          const OperationNode<Base>& indep2,
                                          size_t id2) = 0;

    /**
     * Provides the array name for the temporary variables.
     * It should only be called if temporary variables are saved in an array.
     * 
     * @param var the temporary variable node
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (potentially not unique)
     * @return the array name
     */
    virtual const std::string& getTemporaryVarArrayName(const OperationNode<Base>& var,
                                                        size_t id) = 0;

    /**
     * Provides the index in the associated temporary array of a temporary 
     * variable.
     * It should only be called if temporary variables are saved in an array.
     * 
     * @param var the temporary variable node
     * @param id an ID assigned by the CodeHandler to the operation node
     *           (potentially not unique)
     * @return the index
     */
    virtual size_t getTemporaryVarArrayIndex(const OperationNode<Base>& var,
                                             size_t id) = 0;

    /**
     * Whether or not two temporary variables are considered to be part of
     * the same temporary variable array at consecutive locations.
     * 
     * @param varFirst the temporary variable node with the lower index
     * @param idFirst an ID assigned by the CodeHandler to the first node
     *                (potentially not unique)
     * @param varSecond the temporary variable node with the higher index
     * @param varSecond an ID assigned by the CodeHandler to the second node
     *                  (potentially not unique)
     * @return true if they are consecutive
     */
    virtual bool isConsecutiveInTemporaryVarArray(const OperationNode<Base>& varFirst,
                                                  size_t idFirst,
                                                  const OperationNode<Base>& varSecond,
                                                  size_t idSecond) = 0;

    /**
     * Determines whether or not two temporary variables are part of the same
     * temporary variable array.
     * 
     * @param var1 the temporary variable node
     * @param id1 an ID assigned by the CodeHandler to var1
     *            (potentially not unique)
     * @param var2 the temporary variable node
     * @param id2 an ID assigned by the CodeHandler to var2
     *            (potentially not unique)
     * @return true if the temporary variables are part of the same array
     */
    virtual bool isInSameTemporaryVarArray(const OperationNode<Base>& var1,
                                           size_t id1,
                                           const OperationNode<Base>& var2,
                                           size_t id2) = 0;

    virtual void customFunctionVariableDeclarations(std::ostream& out) {
    }

    virtual void prepareCustomFunctionVariables(std::ostream& out) {
    }

    virtual void finalizeCustomFunctionVariables(std::ostream& out) {
    }

    inline virtual ~VariableNameGenerator() {
    }
};

} // END cg namespace
} // END CppAD namespace

#endif