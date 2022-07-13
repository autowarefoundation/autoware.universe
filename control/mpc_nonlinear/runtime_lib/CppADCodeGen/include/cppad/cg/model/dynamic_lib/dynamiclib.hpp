#ifndef CPPAD_CG_DYNAMICLIB_INCLUDED
#define CPPAD_CG_DYNAMICLIB_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
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
 * Abstract class used to load compiled models in a dynamic library
 * 
 * @author Joao Leal
 */
template<class Base>
class DynamicLib : public FunctorModelLibrary<Base> {
public:

    inline virtual ~DynamicLib() {
    }

};

} // END cg namespace
} // END CppAD namespace

#endif