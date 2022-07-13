#ifndef CPPAD_CG_LANGUAGE_C_FLOAT_INCLUDED
#define CPPAD_CG_LANGUAGE_C_FLOAT_INCLUDED
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

namespace CppAD {
namespace cg {

/**
 * Specialization of the C language function names for floats (requires C99)
 * 
 * @author Joao Leal
 */
template<>
inline const std::string& LanguageC<float>::absFuncName() {
    static const std::string name("fabsf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::acosFuncName() {
    static const std::string name("acosf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::asinFuncName() {
    static const std::string name("asinf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::atanFuncName() {
    static const std::string name("atanf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::coshFuncName() {
    static const std::string name("coshf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::cosFuncName() {
    static const std::string name("cosf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::expFuncName() {
    static const std::string name("expf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::logFuncName() {
    static const std::string name("logf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::sinhFuncName() {
    static const std::string name("sinhf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::sinFuncName() {
    static const std::string name("sinf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::sqrtFuncName() {
    static const std::string name("sqrtf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::tanhFuncName() {
    static const std::string name("tanhf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::tanFuncName() {
    static const std::string name("tanf"); // C99
    return name;
}

#if CPPAD_USE_CPLUSPLUS_2011
template<>
inline const std::string& LanguageC<float>::erfFuncName() {
    static const std::string name("erff"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::asinhFuncName() {
    static const std::string name("asinhf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::acoshFuncName() {
    static const std::string name("acoshf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::atanhFuncName() {
    static const std::string name("atanhf"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::expm1FuncName() {
    static const std::string name("expm1f"); // C99
    return name;
}

template<>
inline const std::string& LanguageC<float>::log1pFuncName() {
    static const std::string name("log1pf"); // C99
    return name;
}

#endif

template<>
inline const std::string& LanguageC<float>::getPrintfBaseFormat() {
    static const std::string format("%f");
    return format;
}

} // END cg namespace
} // END CppAD namespace

#endif