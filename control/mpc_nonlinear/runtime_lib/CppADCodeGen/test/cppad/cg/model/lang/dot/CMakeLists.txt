# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2016 Ciengis
#    Copyright (C) 2020 Joao Leal
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
ADD_DEFINITIONS(-DGRAPHVIZ_DOT_PATH="${GRAPHVIZ_DOT_PATH}")

################################################################################
# add dot test function
################################################################################
FUNCTION(add_cppadcg_dot_test source_file)
    add_cppadcg_test("${source_file}")

    GET_FILENAME_COMPONENT(test_target_name "${source_file}" NAME_WE)
ENDFUNCTION()

################################################################################
# tests
################################################################################
add_cppadcg_dot_test(dot.cpp)
add_cppadcg_dot_test(lang_dot_reset.cpp)
