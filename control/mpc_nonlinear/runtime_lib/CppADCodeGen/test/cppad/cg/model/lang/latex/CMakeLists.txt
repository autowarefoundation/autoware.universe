# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2014 Ciengis
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
ADD_DEFINITIONS(-DPDFLATEX_COMPILER="${PDFLATEX_COMPILER}")

################################################################################
link_file("${CMAKE_CURRENT_SOURCE_DIR}/latexTemplate.tex" "${CMAKE_CURRENT_BINARY_DIR}/latexTemplate.tex")

################################################################################
# add latex test function
################################################################################
FUNCTION(add_cppadcg_latex_test source_file)
    add_cppadcg_test("${source_file}")

    GET_FILENAME_COMPONENT(test_target_name "${source_file}" NAME_WE)

    ADD_CUSTOM_TARGET(link_or_copy_latexTemplate
                      DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/latexTemplate.tex")

    ADD_DEPENDENCIES("${test_target_name}" link_or_copy_latexTemplate)
ENDFUNCTION()

################################################################################
# tests
################################################################################
add_cppadcg_latex_test(latex.cpp)
add_cppadcg_test(lang_latex_reset.cpp)
