#ifndef CPPAD_CG_AUGMENTPATHDEPTHLOOKAHEAD_INCLUDED
#define CPPAD_CG_AUGMENTPATHDEPTHLOOKAHEAD_INCLUDED
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

#include <cppad/cg/dae_index_reduction/augment_path.hpp>

namespace CppAD {
namespace cg {

/**
 * An augment path algorithm based on MC21A.
 * This procedure is depth-first search algorithm with an additional look ahead
 * mechanism to find an unmatched variable for equation node before going
 * deeper.
 */
template<class Base>
class AugmentPathDepthLookahead : public AugmentPath<Base> {
protected:
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
public:

    bool augmentPath(Enode<Base>& i) override final {
        i.color(this->logger_->log(), this->logger_->getVerbosity()); // avoids infinite recursion

        const std::vector<Vnode<Base>*>& vars = i.variables();

        // first look for derivative variables
        for (Vnode<Base>* jj : vars) {
            if (jj->antiDerivative() != nullptr && // not an algebraic variable
                jj->assignmentEquation() == nullptr) { // not assigned yet

                jj->setAssignmentEquation(i, this->logger_->log(), this->logger_->getVerbosity());
                return true;
            }
        }

        // look for algebraic variables
        for (Vnode<Base>* jj : vars) {
            if (jj->antiDerivative() == nullptr &&
                jj->assignmentEquation() == nullptr) { // not assigned yet

                jj->setAssignmentEquation(i, this->logger_->log(), this->logger_->getVerbosity());
                return true;
            }
        }


        for (Vnode<Base>* jj : vars) {
            if (!jj->isColored()) {
                jj->color(this->logger_->log(), this->logger_->getVerbosity());

                Enode<Base>& k = *jj->assignmentEquation(); // all variables are assigned to another equation
                if(!k.isColored()) {
                    bool pathFound = augmentPath(k);
                    if (pathFound) {
                        jj->setAssignmentEquation(i, this->logger_->log(), this->logger_->getVerbosity());
                        return true;
                    }
                }
            }
        }

        return false;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif
