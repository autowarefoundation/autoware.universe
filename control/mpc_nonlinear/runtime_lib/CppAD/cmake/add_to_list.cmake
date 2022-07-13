# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-13 Bradley M. Bell
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
# add_to_list(variable_list constant_value)
#
# variables_list: (in/out)
# The variable containing the list of values. 
# The original list may be ""; i.e., the empty list.
#
# constant_value: (in)
# Is the value we are adding to the list. This value cannot be empty.
#
MACRO(add_to_list variable_list constant_value )
     IF( "${${variable_list}}" STREQUAL "" )
          SET( ${variable_list} ${constant_value} )
     ELSE( "${${variable_list}}" STREQUAL "" )
          SET( ${variable_list} ${${variable_list}} ${constant_value} )
     ENDIF( "${${variable_list}}" STREQUAL "" )
ENDMACRO(add_to_list)
