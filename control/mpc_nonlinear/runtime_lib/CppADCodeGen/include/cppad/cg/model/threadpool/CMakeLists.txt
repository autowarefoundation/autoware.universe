# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2016 Ciengis
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
# files to be installed
# ----------------------------------------------------------------------------
# transform text file into C byte arrays
textfile2h(SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/pthread_pool.c"
		   HEADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/pthread_pool_c.hpp"
		   VARIABLE_NAME "CPPADCG_PTHREAD_POOL_C_FILE")
textfile2h(SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/pthread_pool.h"
		   HEADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/pthread_pool_h.hpp"
		   VARIABLE_NAME "CPPADCG_PTHREAD_POOL_H_FILE")

textfile2h(SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/openmp.c"
		   HEADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/openmp_c.hpp"
		   VARIABLE_NAME "CPPADCG_OPENMP_C_FILE")
textfile2h(SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/openmp.h"
		   HEADER_FILE "${CMAKE_CURRENT_BINARY_DIR}/openmp_h.hpp"
		   VARIABLE_NAME "CPPADCG_OPENMP_H_FILE")

INSTALL( FILES "${CMAKE_CURRENT_BINARY_DIR}/pthread_pool_c.hpp"
		       "${CMAKE_CURRENT_BINARY_DIR}/pthread_pool_h.hpp"
		       "${CMAKE_CURRENT_BINARY_DIR}/openmp_c.hpp"
		       "${CMAKE_CURRENT_BINARY_DIR}/openmp_h.hpp"
		DESTINATION "${install_cppadcg_include_location}/cg/model/threadpool/")