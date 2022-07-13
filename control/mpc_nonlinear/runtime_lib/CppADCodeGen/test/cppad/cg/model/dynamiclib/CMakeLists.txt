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

IF( UNIX )
    add_cppadcg_test(dynamic.cpp)
    add_cppadcg_test(cg_atomic_generic_model.cpp)
    # add_cppadcg_test(dynamic_atomic_nonlinear_outer.cpp) # not supported yet
    add_cppadcg_test(dynamic_atomic.cpp)
    add_cppadcg_test(dynamic_atomic_2.cpp)
    add_cppadcg_test(dynamic_atomic_3.cpp)
    add_cppadcg_test(dynamic_cond_exp.cpp)
    add_cppadcg_test(dynamic_forward_reverse.cpp)
    add_cppadcg_test(dynamic_forward_reverse_2.cpp)
ENDIF()
