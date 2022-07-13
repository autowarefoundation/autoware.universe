# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2013 Ciengis
#    Copyright (C) 2019 Joao Leal
#
#  CppADCodeGen is distributed under multiple licenses:
#
#   - Eclipse Public License Version 1.0 (EPL1), and
#   - GNU General Public License Version 3 (GPL3).
#
#  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
#  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
# ----------------------------------------------------------------------------
#
# Author: Joao Leal
#
# ----------------------------------------------------------------------------
SET(CMAKE_BUILD_TYPE DEBUG)

################################################################################
# tests
################################################################################
add_cppadcg_test(lang_c.cpp)
add_cppadcg_test(lang_c_reset.cpp)
