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

add_cppadcg_test(solve_add.cpp)
add_cppadcg_test(solve_cosh.cpp)
add_cppadcg_test(solve_div.cpp)
add_cppadcg_test(solve_exp.cpp)
add_cppadcg_test(solve_log.cpp)
add_cppadcg_test(solve_log_10.cpp)
add_cppadcg_test(solve_mul.cpp)
add_cppadcg_test(solve_pow.cpp)
add_cppadcg_test(solve_sinh.cpp)
add_cppadcg_test(solve_sqrt.cpp)
add_cppadcg_test(solve_sub.cpp)
add_cppadcg_test(solve_tanh.cpp)
add_cppadcg_test(solve_unary.cpp)

add_cppadcg_test(solve_add_2.cpp)
add_cppadcg_test(solve_div_2.cpp)
add_cppadcg_test(solve_mul_2.cpp)