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
#ifndef CPPAD_CG_CPPADCGINDEXREDUCTIONTEST_HPP
#define	CPPAD_CG_CPPADCGINDEXREDUCTIONTEST_HPP

#include "../../CppADCGTest.hpp"

namespace CppAD {
namespace cg {

class IndexReductionTest : public CppADCGTest {
public:

    inline IndexReductionTest(bool verbose = false, bool printValues = false) :
        CppADCGTest(verbose, printValues) {
        /**
         * disable memory check because CPPAD_USER_ATOMIC is used by the
         * index reduction which creates static variables that will not be
         * destroyed until the program ends
         */
        this->memory_check_ = false;
    }
};

} // END cg namespace
} // END CppAD namespace

#endif
