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

add_cppadcg_test(pattern_matcher.cpp)
add_cppadcg_test(missing_equation.cpp)
add_cppadcg_test(cross_iteration.cpp)
add_cppadcg_test(hessian_with_loops.cpp)
add_cppadcg_test(simple_atomic.cpp)
add_cppadcg_test(simple_atomic_2.cpp)
add_cppadcg_test(simple_atomic_deploop.cpp)
add_cppadcg_test(plug_flow.cpp)
add_cppadcg_test(cstr_collocation.cpp)
add_cppadcg_test(tank_battery.cpp)
#add_cppadcg_test(distillation2.cpp)
#add_cppadcg_test(distillation2_reduced.cpp)
#add_cppadcg_test(distillation.cpp)# takes too long