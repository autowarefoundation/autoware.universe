#ifndef CPPAD_CG_SOARES_SECCHI_HPP
#define CPPAD_CG_SOARES_SECCHI_HPP
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2016 Ciengis
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

#include <cppad/cg/dae_index_reduction/dae_structural_index_reduction.hpp>
#include <cppad/cg/dae_index_reduction/augment_path_depth_lookahead.hpp>
#include <cppad/cg/dae_index_reduction/augment_path_depth_lookahead_a.hpp>

namespace CppAD {
namespace cg {

/**
 * Soares Secchi method for DAE structural index reduction
 */
template<class Base>
class SoaresSecchi : public DaeStructuralIndexReduction<Base> {
protected:
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
protected:
    // avoids having to type this->graph_
    using DaeStructuralIndexReduction<Base>::graph_;
    // typical values used to avoid NaNs in the tape validation by CppAD
    std::vector<Base> x_;
    /**
     * the last equations added to graph
     * (equations used to create the ODE or DAE with index 1)
     */
    std::set<Enode<Base>*> lastAddEq_;
    // whether or not reduceIndex() has been called
    bool reduced_;
    //
    AugmentPathDepthLookahead<Base> defaultAugmentPath_;
    AugmentPathDepthLookaheadA<Base> defaultAugmentPathA_;
    AugmentPath<Base>* augmentPath_;
    AugmentPath<Base>* augmentPathA_;
public:

    /**
     * Creates the DAE index reduction algorithm that implements the
     * Soares Secchi method.
     *
     * @param fun The original model (potentially high index)
     * @param varInfo The DAE system variable information (in the same order
     *                as in the tape)
     * @param eqName Equation names (it can be an empty vector)
     * @param x typical variable values (used to avoid NaNs in CppAD checks)
     */
    SoaresSecchi(ADFun<CG<Base> >& fun,
               const std::vector<DaeVarInfo>& varInfo,
               const std::vector<std::string>& eqName,
               const std::vector<Base>& x) :
            DaeStructuralIndexReduction<Base>(fun, varInfo, eqName),
            x_(x),
            reduced_(false),
            augmentPath_(&defaultAugmentPath_),
            augmentPathA_(&defaultAugmentPathA_){

    }

    SoaresSecchi(const SoaresSecchi& p) = delete;

    SoaresSecchi& operator=(const SoaresSecchi& p) = delete;

    virtual ~SoaresSecchi() {
    }

    AugmentPath<Base>& getAugmentPath() const {
        return *augmentPath_;
    }

    void setAugmentPath(AugmentPath<Base>& a) const {
        augmentPath_ = &a;
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
     * Performs the DAE differentiation index reductions
     *
     * @param newVarInfo Variable related information of the reduced index
     *                   model
     * @param equationInfo Equation related information of the reduced index
     *                     model
     * @return the reduced index model (must be deleted by user)
     * @throws CGException on failure
     */
    inline std::unique_ptr<ADFun<CG<Base>>> reduceIndex(std::vector<DaeVarInfo>& newVarInfo,
                                                        std::vector<DaeEquationInfo>& equationInfo) override {
        if (reduced_)
            throw CGException("reduceIndex() can only be called once!");

        if (this->verbosity_ >= Verbosity::High)
            log() << "########  Soares Secchi method  ########\n";

        augmentPath_->setLogger(*this);
        augmentPathA_->setLogger(*this);

        reduced_ = true;

        // detects which equations have to be differentiated to get an ODE
        detectSubset2Dif();

        // we want an index 1 DAE (ignore the last equations added to the graph)
        for(const Enode<Base>* i: lastAddEq_) {
            graph_.remove(*i);
        }

        if (this->verbosity_ >= Verbosity::Low) {
            graph_.printResultInfo("Soares Secchi");

            log() << "Structural index: " << graph_.getStructuralIndex() << std::endl;
        }

        std::unique_ptr<ADFun<CGBase>> reducedFun(graph_.generateNewModel(newVarInfo, equationInfo, x_));

        return reducedFun;
    }

    /**
     * Provides the differentiation index. It can only be called after
     * reduceIndex().
     *
     * @return the DAE differentiation index.
     * @throws CGException
     */
    inline size_t getStructuralIndex() const {
        return graph_.getStructuralIndex();
    }

protected:
    using DaeStructuralIndexReduction<Base>::log;

    /**
     *
     */
    inline void detectSubset2Dif() {
        auto& vnodes = graph_.variables();
        auto& enodes = graph_.equations();

        std::set<Enode<Base>*> marked;
        std::set<Enode<Base>*> lastMarked;

        if (this->verbosity_ >= Verbosity::High)
            graph_.printDot(this->log());

        while (true) {
            // augment the matching one by one
            for (size_t k = 0; k < enodes.size(); k++) {
                Enode<Base>* i = enodes[k];

                if (this->verbosity_ >= Verbosity::High)
                    log() << "Outer loop: equation k = " << *i << "\n";

                if (i->assignmentVariable() != nullptr) {
                    continue;
                }

                bool pathFound = augmentPathA_->augmentPath(*i);
                if (!pathFound) {

                    for (Enode<Base>* ii: enodes) {
                        // mark colored equations to be differentiated
                        if (ii->isColored() && ii->derivative() == nullptr) {
                            marked.insert(ii);

                            // uncolor equations
                            ii->uncolor();
                        }
                    }

                    pathFound = augmentPath_->augmentPath(*i);
                    if (!pathFound) {
                        throw CGException("Singular system detected.");
                    }

                    for (auto* jj: vnodes)
                        jj->uncolor();

                } else {
                    for (auto* ii: enodes)
                        ii->uncolor();
                }
            }

            if (marked.empty())
                break;

            // diff all MARKED equations
            for (Enode<Base>* i: marked) {
                graph_.createDerivate(*i, false);
            }

            if (this->verbosity_ >= Verbosity::High)
                graph_.printDot(this->log());

            lastMarked.swap(marked);
            marked.clear();
        }

        lastAddEq_.clear();
        for (const Enode<Base>* i: lastMarked) {
            lastAddEq_.insert(i->derivative());
        }

    }

};

} // END cg namespace
} // END CppAD namespace

#endif
