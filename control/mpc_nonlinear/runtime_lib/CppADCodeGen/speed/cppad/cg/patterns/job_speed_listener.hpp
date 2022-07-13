#ifndef CPPAD_CG_JOBSPEEDLISTENER_INCLUDED
#define	CPPAD_CG_JOBSPEEDLISTENER_INCLUDED
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

#include <cppad/cg/cppadcg.hpp>

namespace CppAD {
namespace cg {

class JobSpeedListener : public JobListener {
public:
    using duration = JobListener::duration;
    // pattern detection
    duration patternDection;
    // pattern detection
    duration graphGen;
    // total time used for source code generation
    duration srcCodeGen;
    // source code compilation
    duration srcCodeComp;
    // compilation of the dynamic library
    duration dynLibComp;
    // JIT preparation time
    duration jit;
    // total time used to compile the sources and generate the library
    duration totalLibrary;
public:
    JobSpeedListener();

    inline void reset() {
        patternDection = duration(0);
        graphGen = duration(0);
        srcCodeGen = duration(0);
        srcCodeComp = duration(0);
        dynLibComp = duration(0);
        jit = duration(0);
        totalLibrary = duration(0);
    }

    void jobStarted(const std::vector<Job>& job) override;

    void jobEndended(const std::vector<Job>& job,
                     duration elapsed) override;
};

} // END cg namespace
} // END CppAD namespace

#endif
