#ifndef CPPAD_CG_THREAD_POOL_SCHEDULE_STRATEGY_INCLUDED
#define CPPAD_CG_THREAD_POOL_SCHEDULE_STRATEGY_INCLUDED
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

enum class ThreadPoolScheduleStrategy {
    STATIC = 1, // all jobs are assigned to a thread at the beginning
    DYNAMIC = 2, // each thread only executes a single job at a time
    GUIDED = 3 // each thread can execute multiple jobs before returning to the pool
};

}
}

#endif