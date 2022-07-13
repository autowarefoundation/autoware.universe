# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2013 Ciengis
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

add_cppadcg_test(pantelides.cpp)
add_cppadcg_test(pantelides_flash.cpp)

add_cppadcg_test(soares_secchi.cpp)
add_cppadcg_test(soares_secchi_flash.cpp)
add_cppadcg_test(soares_secchi_destil.cpp)

IF(EIGEN3_FOUND)
  add_cppadcg_test(dummy_derivative.cpp)
  add_cppadcg_test(dummy_derivative_destil.cpp)
  add_cppadcg_test(dummy_derivative_linear.cpp)
ELSE()
  MESSAGE(WARNING 'Eigen3 not found: Dummy derivatives tests disabled!')
ENDIF()