# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2015 Ciengis
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
# tests
################################################################################
add_cppadcg_test(mathml.cpp)
add_cppadcg_test(lang_mathml_reset.cpp)

link_file("${CMAKE_CURRENT_SOURCE_DIR}/variableSelection.js"
          "${CMAKE_CURRENT_BINARY_DIR}/variableSelection.js")
link_file("${CMAKE_CURRENT_SOURCE_DIR}/mathml.css"
          "${CMAKE_CURRENT_BINARY_DIR}/mathml.css")
link_file("${CMAKE_CURRENT_SOURCE_DIR}/head_extra.html"
          "${CMAKE_CURRENT_BINARY_DIR}/head_extra.html")


ADD_CUSTOM_TARGET(link_or_copy_variableSelectionJS
                  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/variableSelection.js"
                          "${CMAKE_CURRENT_BINARY_DIR}/mathml.css"
                          "${CMAKE_CURRENT_BINARY_DIR}/head_extra.html")

ADD_DEPENDENCIES("mathml" link_or_copy_variableSelectionJS)