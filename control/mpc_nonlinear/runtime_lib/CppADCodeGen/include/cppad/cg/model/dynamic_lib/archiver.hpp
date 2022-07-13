#ifndef CPPAD_CG_ARCHIVER_INCLUDED
#define CPPAD_CG_ARCHIVER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

#include <typeinfo>

namespace CppAD {
namespace cg {

/**
 * A tool used to create static libraries from object files
 */
class Archiver {
public:
    virtual bool isVerbose() const = 0;

    virtual void setVerbose(bool verbose) = 0;

    virtual void create(const std::string& library,
                        const std::set<std::string>& objectFiles,
                        JobTimer* timer = nullptr) = 0;

    inline virtual ~Archiver() {
    };
};

} // END cg namespace
} // END CppAD namespace

#endif