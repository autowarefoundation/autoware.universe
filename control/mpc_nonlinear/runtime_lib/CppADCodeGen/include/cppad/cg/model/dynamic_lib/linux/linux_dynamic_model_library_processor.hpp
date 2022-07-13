#ifndef CPPAD_CG_LINUX_C_LANG_COMPILE_DYNAMIC_HELPER_INCLUDED
#define CPPAD_CG_LINUX_C_LANG_COMPILE_DYNAMIC_HELPER_INCLUDED
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
#if CPPAD_CG_SYSTEM_LINUX

namespace CppAD {
namespace cg {

template<class Base>
std::unique_ptr<DynamicLib<Base>> DynamicModelLibraryProcessor<Base>::loadDynamicLibrary() {
    std::unique_ptr<DynamicLib<Base>> lib;
    const auto it = _options.find("dlOpenMode");
    if (it == _options.end()) {
        lib.reset(new LinuxDynamicLib<Base>(_libraryName + system::SystemInfo<>::DYNAMIC_LIB_EXTENSION));
    } else {
        int dlOpenMode = std::stoi(it->second);
        lib.reset(new LinuxDynamicLib<Base>(_libraryName + system::SystemInfo<>::DYNAMIC_LIB_EXTENSION, dlOpenMode));
    }
    return lib;
}

} // END cg namespace
} // END CppAD namespace

#endif
#endif
