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

IF( UNIX )
  ADD_LIBRARY(pthread_pool STATIC
              ${CMAKE_SOURCE_DIR}/include/cppad/cg/model/threadpool/pthread_pool.c)

  add_cppadcg_test(pthreadpool_raw)

  TARGET_LINK_LIBRARIES(pthreadpool_raw pthread_pool)
  TARGET_LINK_LIBRARIES(pthreadpool_raw ${CMAKE_THREAD_LIBS_INIT} )

ENDIF()

add_cppadcg_test(dynamiclib_pthreadpool.cpp)
IF (OPENMP_FOUND)
  #add_cppadcg_test(dynamiclib_openmp.cpp) # disabled until OpenMP allows libraries to be loaded dynamically and then gracefully closed
ENDIF()
