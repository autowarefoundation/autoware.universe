# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2012 Ciengis
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

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

add_cppadcg_test(abs.cpp)
add_cppadcg_test(acos.cpp)
add_cppadcg_test(acosh.cpp)
add_cppadcg_test(add.cpp)
add_cppadcg_test(asin.cpp)
add_cppadcg_test(asinh.cpp)
add_cppadcg_test(assign.cpp)
add_cppadcg_test(atan.cpp)
add_cppadcg_test(atan_2.cpp)
add_cppadcg_test(atanh.cpp)
add_cppadcg_test(compare_change.cpp)
add_cppadcg_test(cond_exp.cpp)
add_cppadcg_test(cos.cpp)
add_cppadcg_test(cosh.cpp)
add_cppadcg_test(div.cpp)
add_cppadcg_test(erf.cpp)
add_cppadcg_test(erfc.cpp)
add_cppadcg_test(exp.cpp)
add_cppadcg_test(expm1.cpp)
add_cppadcg_test(log.cpp)
add_cppadcg_test(log1p.cpp)
add_cppadcg_test(log_10.cpp)
add_cppadcg_test(mul.cpp)
add_cppadcg_test(parameter.cpp)
add_cppadcg_test(pow.cpp)
add_cppadcg_test(sin.cpp)
add_cppadcg_test(sub.cpp)
add_cppadcg_test(tan.cpp)
add_cppadcg_test(unary.cpp)
