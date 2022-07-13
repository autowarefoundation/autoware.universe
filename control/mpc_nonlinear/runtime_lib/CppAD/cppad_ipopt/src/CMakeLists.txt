# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the cppad_ipopt/src library
# Inherit build type from ../CMakeList.txt

#  add_library(<name> [STATIC | SHARED | MODULE] [EXCLUDE_FROM_ALL]
#   source1 source2 ... sourceN)
# )
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    cppad_ipopt_nlp.cpp
    hes_fg_map.cpp
    jac_g_map.cpp
    sparse_map2vec.cpp
    vec_fun_pattern.cpp
)
# END_SORT_THIS_LINE_MINUS_2

ADD_LIBRARY(cppad_ipopt ${source_list})
set_compile_flags( cppad_ipopt "${cppad_debug_which}" "${source_list}" )

# install(FILES files... DESTINATION <dir>
#   [PERMISSIONS permissions...]
#   [CONFIGURATIONS [Debug|Release|...]]
#   [COMPONENT <component>]
#   [RENAME <name>] [OPTIONAL])
INSTALL(FILES cppad_ipopt_nlp.hpp DESTINATION ${cppad_abs_includedir})

# install(TARGETS myExe mySharedLib myStaticLib
#   RUNTIME DESTINATION bin
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib/static)
INSTALL(TARGETS cppad_ipopt DESTINATION ${cppad_abs_libdir})
