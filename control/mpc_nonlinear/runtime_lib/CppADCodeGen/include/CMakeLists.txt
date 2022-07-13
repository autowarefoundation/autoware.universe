# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2012 Ciengis
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

CONFIGURE_FILE(
		${CMAKE_CURRENT_SOURCE_DIR}/cppad/cg/configure.hpp.in
		${CMAKE_CURRENT_BINARY_DIR}/cppad/cg/configure.hpp
)
INSTALL( FILES ${CMAKE_CURRENT_BINARY_DIR}/cppad/cg/configure.hpp
         DESTINATION "${install_cppadcg_include_location}/cg")

# ----------------------------------------------------------------------------
# files to be installed
# ----------------------------------------------------------------------------

INSTALL( DIRECTORY "cppad/cg/"
         DESTINATION "${install_cppadcg_include_location}/cg"
         FILES_MATCHING PATTERN "*.hpp")

INSTALL(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cppad/cg.hpp"
	    DESTINATION "${install_cppadcg_include_location}/" )

ADD_SUBDIRECTORY(cppad/cg/model/threadpool)