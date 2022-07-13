#ifndef CPPAD_CG_LANG_C_ATOMIC_FUN_INCLUDED
#define CPPAD_CG_LANG_C_ATOMIC_FUN_INCLUDED
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

extern "C" {

/**
 * A wrapper for an array.
 * It is used to call forward and reverse functions of atomic functions in
 * the generated C source-code.
 */
typedef struct Array {
    /**
     * Array values. For dense arrays its size is defined by
     * ::size otherwise by ::nnz.
     */
    void* data;
    /**
     * Total array size.
     */
    unsigned long size;
    /**
     * Whether or not it is a sparse array.
     */
    int sparse;
    /**
     * Indexes of sparse array (undefined for dense).
     */
    const unsigned long* idx;
    /**
     * Number of non-zeros (size of data; undefined for dense).
     */
    unsigned long nnz;
} Array;

/**
 * Holds function pointers that the compiled code uses to call atomic functions.
 */
struct LangCAtomicFun {
    /**
     * A pointer to the compiled model object (e.g. LinuxDynamicLibModel)
     */
    void* libModel;

    int (*forward)(void* libModel,
                   int atomicIndex,
                   int q,
                   int p,
                   const Array tx[],
                   Array* ty);

    int (*reverse)(void* libModel,
                   int atomicIndex,
                   int p,
                   const Array tx[],
                   Array* px,
                   const Array py[]);
};

}

#endif
