# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Build the example/general directory tests
#
# adolc_sources
IF( cppad_has_adolc )
    SET(adolc_sources mul_level_adolc.cpp mul_level_adolc_ode.cpp)
ELSE( cppad_has_adolc )
    SET(adolc_sources "")
ENDIF( cppad_has_adolc )
#
# eigen_sources
IF( cppad_has_eigen )
    SET(eigen_sources eigen_det.cpp eigen_array.cpp)
ELSE( cppad_has_eigen )
    SET(eigen_sources "")
ENDIF( cppad_has_eigen )
#
# BEGIN_SORT_THIS_LINE_PLUS_2
SET(source_list
    ${adolc_sources}
    ${eigen_sources}
    abort_recording.cpp
    acos.cpp
    acosh.cpp
    ad_assign.cpp
    ad_ctor.cpp
    ad_fun.cpp
    ad_in_c.cpp
    ad_input.cpp
    ad_output.cpp
    add.cpp
    add_eq.cpp
    asin.cpp
    asinh.cpp
    atan.cpp
    atan2.cpp
    atanh.cpp
    azmul.cpp
    base2ad.cpp
    base_alloc.hpp
    base_require.cpp
    bender_quad.cpp
    bool_fun.cpp
    capacity_order.cpp
    change_param.cpp
    check_for_nan.cpp
    compare.cpp
    compare_change.cpp
    complex_poly.cpp
    con_dyn_var.cpp
    cond_exp.cpp
    cos.cpp
    cosh.cpp
    div.cpp
    div_eq.cpp
    equal_op_seq.cpp
    erf.cpp
    erfc.cpp
    exp.cpp
    expm1.cpp
    fabs.cpp
    for_one.cpp
    for_two.cpp
    forward.cpp
    forward_dir.cpp
    forward_order.cpp
    fun_assign.cpp
    fun_check.cpp
    fun_property.cpp
    function_name.cpp
    general.cpp
    hes_lagrangian.cpp
    hes_lu_det.cpp
    hes_minor_det.cpp
    hes_times_dir.cpp
    hessian.cpp
    independent.cpp
    integer.cpp
    interface2c.cpp
    interp_onetape.cpp
    interp_retape.cpp
    jac_lu_det.cpp
    jac_minor_det.cpp
    jacobian.cpp
    log.cpp
    log10.cpp
    log1p.cpp
    lu_ratio.cpp
    lu_vec_ad.cpp
    lu_vec_ad.hpp
    lu_vec_ad_ok.cpp
    mul.cpp
    mul_eq.cpp
    mul_level.cpp
    mul_level_ode.cpp
    near_equal_ext.cpp
    new_dynamic.cpp
    num_limits.cpp
    number_skip.cpp
    numeric_type.cpp
    ode_stiff.cpp
    opt_val_hes.cpp
    pow.cpp
    pow_nan.cpp
    print_for.cpp
    rev_checkpoint.cpp
    rev_one.cpp
    rev_two.cpp
    reverse_one.cpp
    reverse_three.cpp
    reverse_two.cpp
    sign.cpp
    sin.cpp
    sinh.cpp
    sqrt.cpp
    stack_machine.cpp
    sub.cpp
    sub_eq.cpp
    tan.cpp
    tanh.cpp
    tape_index.cpp
    taylor_ode.cpp
    unary_minus.cpp
    unary_plus.cpp
    value.cpp
    var2par.cpp
    vec_ad.cpp
)
# END_SORT_THIS_LINE_MINUS_2

set_compile_flags( example_general "${cppad_debug_which}" "${source_list}" )
#
ADD_EXECUTABLE(example_general EXCLUDE_FROM_ALL ${source_list})
#
# List of libraries to be linked into the specified target
TARGET_LINK_LIBRARIES(example_general
    ${adolc_LIBRARIES}
)
#
# Add the check_example_general target
ADD_CUSTOM_TARGET(check_example_general
    example_general
    DEPENDS example_general
)
MESSAGE(STATUS "make check_example_general: available")
#
# add to check check_example_depends in parent environment
add_to_list(check_example_depends check_example_general)
SET(check_example_depends "${check_example_depends}" PARENT_SCOPE)
