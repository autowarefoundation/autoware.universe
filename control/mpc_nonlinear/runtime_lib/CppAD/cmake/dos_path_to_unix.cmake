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
# dos_path_to_unix(dos_path unix_path)
#
# dos_path: (in)
# is the value of the path we are converting to unix format; i.e.,
# all \ characters are replaced by / characters.
#
# unix_path: (out)
# is the variable where the result of the conversion is placed.
# 
MACRO(dos_path_to_unix dos_path unix_path)
    STRING(REGEX REPLACE "[\\]" "/" ${unix_path} "${dos_path}" )
ENDMACRO(dos_path_to_unix dos_path unix_path)
