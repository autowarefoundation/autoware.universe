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
# assert_value_in_set(value element_1 ... element_n )
#
# value: (in)
# is the value we are searching for.
#
# element_j: (in)
# is the j-th element of the set.
#
MACRO(assert_value_in_set)
    SET(argv ${ARGV})
    LIST(GET argv 0 value)
    LIST(REMOVE_AT argv 0)
    SET(ok FALSE)
    FOREACH(entry ${argv} )
        IF( "${${value}}" STREQUAL "${entry}" )
            SET(ok TRUE)
        ENDIF( "${${value}}" STREQUAL "${entry}" )
    ENDFOREACH(entry ${argv} )
    IF(NOT ok)
        MESSAGE(FATAL_ERROR
            "${value} is not one of following: ${argv}"
        )
    ENDIF(NOT ok)
ENDMACRO(assert_value_in_set)
