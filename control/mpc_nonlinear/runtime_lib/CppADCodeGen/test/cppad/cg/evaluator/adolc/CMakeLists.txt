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

# includes
INCLUDE_DIRECTORIES(${ADOLC_INCLUDE_DIRS})

# add adolc dependency
FUNCTION(add_cppadcg_test_adolc source_file)
    add_cppadcg_test("${source_file}")

    GET_FILENAME_COMPONENT(test_target_name "${source_file}" NAME_WE)
    TARGET_LINK_LIBRARIES(${test_target_name} ${ADOLC_LIBRARIES})
ENDFUNCTION()

add_cppadcg_test_adolc(evaluator_adolc_add.cpp)
add_cppadcg_test_adolc(evaluator_adolc_cosh.cpp)
add_cppadcg_test_adolc(evaluator_adolc_div.cpp)
add_cppadcg_test_adolc(evaluator_adolc_exp.cpp)
add_cppadcg_test_adolc(evaluator_adolc_log.cpp)
add_cppadcg_test_adolc(evaluator_adolc_log_10.cpp)
add_cppadcg_test_adolc(evaluator_adolc_mul.cpp)
add_cppadcg_test_adolc(evaluator_adolc_pow.cpp)
add_cppadcg_test_adolc(evaluator_adolc_sinh.cpp)
add_cppadcg_test_adolc(evaluator_adolc_sqrt.cpp)
add_cppadcg_test_adolc(evaluator_adolc_sub.cpp)
add_cppadcg_test_adolc(evaluator_adolc_tanh.cpp)
add_cppadcg_test_adolc(evaluator_adolc_unary.cpp)
