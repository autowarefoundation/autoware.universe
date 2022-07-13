#ifndef CPPAD_CG_MULTITHREADINGTYPE_INCLUDED
#define CPPAD_CG_MULTITHREADINGTYPE_INCLUDED
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

enum class MultiThreadingType {
    NONE, // no multithreading
    OPENMP, // using the OpenMP library (does not work on dynamically loaded model libraries)
    PTHREADS // using the PThreads library
};

}
}

#endif
