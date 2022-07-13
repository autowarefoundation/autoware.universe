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
#include "job_speed_listener.hpp"

using namespace CppAD::cg;

JobSpeedListener::JobSpeedListener() :
    patternDection(0),
    graphGen(0),
    srcCodeGen(0),
    srcCodeComp(0),
    dynLibComp(0),
    jit(0),
    totalLibrary(0) {
}

void JobSpeedListener::jobStarted(const std::vector<Job>& job) {
    // do nothing
}

void JobSpeedListener::jobEndended(const std::vector<Job>& job,
                                   std::chrono::steady_clock::duration elapsed) {
    const Job& j = job.back();

    if (&j.getType() == &JobTimer::LOOP_DETECTION) {
        // pattern detection
        patternDection = elapsed;
    } else if (&j.getType() == &JobTimer::GRAPH) {
        graphGen += elapsed;
    } else if (&j.getType() == &JobTimer::SOURCE_FOR_MODEL) {
        // source code generation
        srcCodeGen = elapsed;
    } else if (&j.getType() == &JobTimer::COMPILING_FOR_MODEL) {
        // source code compilation
        srcCodeComp = elapsed;
    } else if (&j.getType() == &JobTimer::SOURCE_GENERATION) {
        // individual source files
    } else if (&j.getType() == &JobTimer::COMPILING_DYNAMIC_LIBRARY) {
        dynLibComp = elapsed;
    } else if (&j.getType() == &JobTimer::DYNAMIC_MODEL_LIBRARY) {
        totalLibrary = elapsed;
    } else if (&j.getType() == &JobTimer::JIT_MODEL_LIBRARY) {
        jit = elapsed;
    }

}
