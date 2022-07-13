#ifndef CPPAD_CG_DAE_STRUCTURAL_INDEX_REDUCTION_INCLUDED
#define CPPAD_CG_DAE_STRUCTURAL_INDEX_REDUCTION_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

#include <cppad/cg/dae_index_reduction/dae_index_reduction.hpp>
#include <cppad/cg/dae_index_reduction/bipartite_graph.hpp>

namespace CppAD {
namespace cg {

/**
 * Base class for algorithms that perform automatic
 * structural index reduction of implicit DAEs using graphs.
 */
template<class Base>
class DaeStructuralIndexReduction : public DaeIndexReduction<Base> {
protected:
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
protected:
    //
    BipartiteGraph<Base> graph_;
public:

    /**
     * Creates a new algorithm for structural index reduction of DAE systems.
     *
     * @param fun The original model (potentially high index)
     * @param varInfo The DAE system variable information (in the same order
     *                as in the tape)
     * @param eqName Equation names (it can be an empty vector)
     */
    DaeStructuralIndexReduction(ADFun<CG<Base>>& fun,
                                const std::vector<DaeVarInfo>& varInfo,
                                const std::vector<std::string>& eqName) :
            DaeIndexReduction<Base>(fun),
            graph_(fun, varInfo, eqName, *this) {
    }

    inline virtual ~DaeStructuralIndexReduction() {
    }

    inline BipartiteGraph<Base>& getGraph() {
        return graph_;
    }

    inline const BipartiteGraph<Base>& getGraph() const {
        return graph_;
    }

    /**
     * Defines whether or not original names saved by using
     * CppAD::PrintFor(0, "", val, name)
     * should be kept by also adding PrintFor operations in the reduced model.
     */
    inline void setPreserveNames(bool p) {
        graph_.setPreserveNames(p);
    }

    /**
     * Whether or not original names saved by using
     * CppAD::PrintFor(0, "", val, name)
     * should be kept by also adding PrintFor operations in the reduced model.
     */
    inline bool isPreserveNames() const {
        return graph_.isPreserveNames();
    }

    /**
     * Provides the structural index which is typically a good approximation of
     * the differentiation index.
     * It can only be called after reduceIndex().
     *
     * @return the DAE structural index.
     * @throws CGException
     */
    inline size_t getStructuralIndex() const {
        return graph_.getStructuralIndex();
    }
};

} // END cg namespace
} // END CppAD namespace

#endif	

