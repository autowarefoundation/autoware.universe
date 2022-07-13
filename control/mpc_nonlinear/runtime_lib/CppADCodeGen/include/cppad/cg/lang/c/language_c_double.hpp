#ifndef CPPAD_CG_LANGUAGE_C_DOUBLE_INCLUDED
#define CPPAD_CG_LANGUAGE_C_DOUBLE_INCLUDED
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
 * Specialization of the C language function names for doubles
 * 
 * @author Joao Leal
 */
template<>
inline const std::string& LanguageC<double>::absFuncName() {
    static const std::string name("fabs");
    return name;
}

template<>
inline const std::string& LanguageC<double>::getPrintfBaseFormat() {
    static const std::string format("%f");
    return format;
}

} // END cg namespace
} // END CppAD namespace

#endif