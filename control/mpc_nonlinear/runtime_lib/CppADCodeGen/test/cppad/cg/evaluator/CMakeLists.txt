# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2014 Ciengis
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

add_cppadcg_test(evaluator_add.cpp)
add_cppadcg_test(evaluator_cosh.cpp)
add_cppadcg_test(evaluator_div.cpp)
add_cppadcg_test(evaluator_exp.cpp)
add_cppadcg_test(evaluator_log.cpp)
add_cppadcg_test(evaluator_log_10.cpp)
add_cppadcg_test(evaluator_mul.cpp)
add_cppadcg_test(evaluator_pow.cpp)
add_cppadcg_test(evaluator_sinh.cpp)
add_cppadcg_test(evaluator_sqrt.cpp)
add_cppadcg_test(evaluator_sub.cpp)
add_cppadcg_test(evaluator_tanh.cpp)
add_cppadcg_test(evaluator_unary.cpp)

IF(ADOLC_FOUND)
    ADD_SUBDIRECTORY(adolc)
ENDIF()
ADD_SUBDIRECTORY(adcg)
ADD_SUBDIRECTORY(cg)