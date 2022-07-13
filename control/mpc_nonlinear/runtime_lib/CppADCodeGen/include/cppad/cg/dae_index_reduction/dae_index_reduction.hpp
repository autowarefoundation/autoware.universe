#ifndef CPPAD_CG_DAE_INDEX_REDUCTION_INCLUDED
#define CPPAD_CG_DAE_INDEX_REDUCTION_INCLUDED
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

#include <cppad/cg/cppadcg.hpp>
#include <cppad/cg/dae_index_reduction/dae_var_info.hpp>
#include <cppad/cg/dae_index_reduction/dae_equation_info.hpp>
#include <cppad/cg/dae_index_reduction/simple_logger.hpp>

namespace CppAD {
namespace cg {

/**
 * Base class for algorithms that perform automatic index reduction of
 * implicit DAEs.
 */
template<class Base>
class DaeIndexReduction : public SimpleLogger {
protected:
    /**
     * The original model representing an implicit DAE system
     */
    ADFun<CG<Base> >* const fun_;
public:

    /**
     * Creates a new algorithm for index reduction of DAE systems.
     * 
     * @param fun The original model (potentially high index)
     */
    DaeIndexReduction(ADFun<CG<Base> >& fun) :
        fun_(&fun) {
    }

    inline virtual ~DaeIndexReduction() {
    }

    /**
     * Provides the original model with a representation of an implicit DAE
     * (potentially high index).
     */
    inline ADFun<CG<Base> >& getOriginalModel() const {
        return *fun_;
    }

    /**
     * Performs the DAE index reduction and creates a new reduced index model.
     *
     * @param newVarInfo Variable related information of the reduced index model
     * @param equationInfo Equation related information of the reduced index model
     * @return the reduced index model
     *         (null if there was no need for index reduction)
     */
    virtual std::unique_ptr<ADFun<CG<Base>>> reduceIndex(std::vector<DaeVarInfo>& newVarInfo,
                                                         std::vector<DaeEquationInfo>& equationInfo) = 0;

};

} // END cg namespace
} // END CppAD namespace

#endif	

