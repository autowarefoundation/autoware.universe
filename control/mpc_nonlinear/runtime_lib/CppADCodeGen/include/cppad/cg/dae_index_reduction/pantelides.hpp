#ifndef CPPAD_CG_PANTELIDES_INCLUDED
#define CPPAD_CG_PANTELIDES_INCLUDED
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

#include <cppad/cg/dae_index_reduction/dae_structural_index_reduction.hpp>
#include <cppad/cg/dae_index_reduction/augment_path_depth_lookahead.hpp>

namespace CppAD {
namespace cg {

/**
 * Pantelides DAE index reduction algorithm
 */
template<class Base>
class Pantelides : public DaeStructuralIndexReduction<Base> {
protected:
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
protected:
    // avoids having to type this->graph_
    using DaeStructuralIndexReduction<Base>::graph_;
    // typical values used to avoid NaNs in the tape validation by CppAD
    std::vector<Base> x_;
    // whether or not reduceIndex() has been called
    bool reduced_;
    AugmentPathDepthLookahead<Base> defaultAugmentPath_;
    AugmentPath<Base>* augmentPath_;
public:

    /**
     * Creates the DAE index reduction algorithm that implements the
     * Pantelides method.
     *
     * @param fun The original model (potentially high index)
     * @param varInfo The DAE system variable information (in the same order
     *                as in the tape)
     * @param eqName Equation names (it can be an empty vector)
     * @param x Typical variable values (used to avoid NaNs in CppAD checks)
     */
    Pantelides(ADFun<CG<Base> >& fun,
               const std::vector<DaeVarInfo>& varInfo,
               const std::vector<std::string>& eqName,
               const std::vector<Base>& x) :
            DaeStructuralIndexReduction<Base>(fun, varInfo, eqName),
            x_(x),
            reduced_(false),
            augmentPath_(&defaultAugmentPath_) {

    }

    Pantelides(const Pantelides& p) = delete;

    Pantelides& operator=(const Pantelides& p) = delete;

    virtual ~Pantelides() = default;

    AugmentPath<Base>& getAugmentPath() const {
        return *augmentPath_;
    }

    void setAugmentPath(AugmentPath<Base>& a) const {
        augmentPath_ = &a;
    }

    inline std::unique_ptr<ADFun<CG<Base>>> reduceIndex(std::vector<DaeVarInfo>& newVarInfo,
                                                        std::vector<DaeEquationInfo>& equationInfo) override {
        if (reduced_)
            throw CGException("reduceIndex() can only be called once!");

        if (this->verbosity_ >= Verbosity::Low)
            log() << "########  Pantelides method  ########\n";

        augmentPath_->setLogger(*this);

        reduced_ = true;

        detectSubset2Dif();

        if (this->verbosity_ >= Verbosity::Low) {
            graph_.printResultInfo("Pantelides");

            log() << "Structural index: " << graph_.getStructuralIndex() << std::endl;
        }

        std::unique_ptr<ADFun<CGBase>> reducedFun(graph_.generateNewModel(newVarInfo, equationInfo, x_));

        return reducedFun;
    }

protected:
    using DaeStructuralIndexReduction<Base>::log;

    /**
     *
     */
    inline void detectSubset2Dif() {
        auto& vnodes = graph_.variables();
        auto& enodes = graph_.equations();

        Enode<Base>* ll;

        if (this->verbosity_ >= Verbosity::High)
            graph_.printDot(this->log());

        size_t Ndash = enodes.size();
        for (size_t k = 0; k < Ndash; k++) {
            Enode<Base>* i = enodes[k];

            if (this->verbosity_ >= Verbosity::High)
                log() << "Outer loop: equation k = " << *i << "\n";

            bool pathfound = false;
            while (!pathfound) {

                /**
                 * delete all V-nodes with A!=0 and their incident edges
                 * from the graph
                 */
                for (Vnode<Base>* jj : vnodes) {
                    if (!jj->isDeleted() && jj->derivative() != nullptr) {
                        jj->deleteNode(log(), this->verbosity_);
                    }
                }

                graph_.uncolorAll();

                pathfound = augmentPath_->augmentPath(*i);

                if (!pathfound) {
                    const size_t vsize = vnodes.size(); // the size might change
                    for (size_t l = 0; l < vsize; ++l) {
                        Vnode<Base>* jj = vnodes[l];
                        if (jj->isColored() && !jj->isDeleted()) {
                            // add new variable derivatives of colored variables
                            graph_.createDerivate(*jj);
                        }
                    }

                    const size_t esize = enodes.size(); // the size might change
                    for (size_t l = 0; l < esize; l++) {
                        ll = enodes[l];
                        if (ll->isColored()) {
                            // add new derivative equations for colored equations and create edges
                            graph_.createDerivate(*ll);
                        }
                    }

                    // structural check to avoid infinite recursion
                    for (size_t l = esize; l < enodes.size(); l++) {
                        ll = enodes[l];
                        const std::vector<Vnode<Base>*>& nvars = ll->originalVariables();
                        bool ok = false;
                        for (Vnode<Base>* js : nvars) {
                            if (js->equations().size() > 1) {
                                ok = true;
                                break;
                            }
                        }
                        if (!ok)
                            throw CGException("Invalid equation structure. The model appears to be over-defined.");
                    }

                    for (Vnode<Base>* jj : vnodes) {
                        if (jj->isColored() && !jj->isDeleted()) {
                            Vnode<Base>* jDiff = jj->derivative();
                            jDiff->setAssignmentEquation(*jj->assignmentEquation()->derivative(), log(), this->verbosity_);
                        }
                    }

                    i = i->derivative();

                    if (this->verbosity_ >= Verbosity::High) {
                        log() << "Set current equation to (i=" << i->index() << ") " << *i << "\n";

                        graph_.printDot(this->log());
                    }
                }
            }

        }

    }

};

} // END cg namespace
} // END CppAD namespace

#endif
