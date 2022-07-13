# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# =============================================================================
# assert(variable)
#
# variable: (in)
# The variable is checked to make sure it is true, and if it is not true
# a fatal error message is printed.
#
MACRO(assert variable)
     IF( NOT ${variable} )
        MESSAGE(FATAL_ERROR
            "Error: ${variable} is false in ${CMAKE_CURRENT_LIST_FILE}"
        )
     ENDIF( NOT ${variable} )
ENDMACRO(assert)
