#ifndef CPPAD_CG_AUGMENTPATH_INCLUDED
#define CPPAD_CG_AUGMENTPATH_INCLUDED
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

#include <cppad/cg/dae_index_reduction/bipartite_nodes.hpp>
#include <cppad/cg/dae_index_reduction/dae_equation_info.hpp>
#include <cppad/cg/dae_index_reduction/simple_logger.hpp>

namespace CppAD {
namespace cg {

/**
 * Algorithm interface for assigning equations to variables in sorting
 * procedures.
 */
template<class Base>
class AugmentPath {
protected:
    using CGBase = CppAD::cg::CG<Base>;
    using ADCG = CppAD::AD<CGBase>;
protected:
    SimpleLogger defaultLogger_;
    // logger
    SimpleLogger* logger_;
public:
    inline AugmentPath() :
            logger_(&defaultLogger_) {
    }

    inline virtual ~AugmentPath() {
    }

    /**
     *
     * @param i The equation node
     * @return true if an augmented path was found
     */
    virtual bool augmentPath(Enode<Base>& i) = 0;

    inline void setLogger(SimpleLogger& logger) {
        logger_ = &logger;
    }

    inline SimpleLogger& getLogger() const {
        return *logger_;
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
