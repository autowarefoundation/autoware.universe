#ifndef CPPAD_CG_OSTREAM_CONFIG_RESTORE_INCLUDED
#define CPPAD_CG_OSTREAM_CONFIG_RESTORE_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2015 Ciengis
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

#include <iostream>

namespace CppAD {
namespace cg {

class OStreamConfigRestore {
private:
    std::ostream& os;
    std::ios::fmtflags f;
    std::streamsize nf;
    std::streamsize nw;
public:

    inline explicit OStreamConfigRestore(std::ostream& os) :
        os(os),
        f(os.flags()),
        nf(os.precision()),
        nw(os.width()) {
    }

    OStreamConfigRestore(const OStreamConfigRestore &rhs) = delete;
    OStreamConfigRestore& operator=(const OStreamConfigRestore& rhs) = delete;

    inline ~OStreamConfigRestore() {
        os.flags(f);
        os.precision(nf);
        os.width(nw);
    }
};

} // END cg namespace
} // END CppAD namespace

#endif