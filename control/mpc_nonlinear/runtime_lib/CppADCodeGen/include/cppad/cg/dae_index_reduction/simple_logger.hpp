#ifndef CPPAD_CG_SIMPLE_LOGGER_INCLUDED
#define CPPAD_CG_SIMPLE_LOGGER_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * A very simple logger
 */
class SimpleLogger {
protected:
    // verbosity level
    Verbosity verbosity_;
    // output stream used for logging
    std::ostream* log_;
public:

    /**
     * Creates a new simple logger
     */
    inline SimpleLogger() :
            verbosity_(Verbosity::None),
            log_(&std::cout) {
    }

    inline virtual ~SimpleLogger() {
    }

    inline std::ostream& log() const {
        return *log_;
    }

    inline void setLog(std::ostream& out) {
        log_ = &out;
    }

    inline void setVerbosity(Verbosity verbosity) {
        verbosity_ = verbosity;
    }

    inline Verbosity getVerbosity() const {
        return verbosity_;
    }

};

} // END cg namespace
} // END CppAD namespace

#endif	

