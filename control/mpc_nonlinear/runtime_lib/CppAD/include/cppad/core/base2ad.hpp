# ifndef CPPAD_CORE_BASE2AD_HPP
# define CPPAD_CORE_BASE2AD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin base2ad$$
$spell
    af
    Taylor
$$

$spell
$$

$section Create an AD<Base> Function From a Base Function$$

$head Syntax$$
$icode%af% = %f%.base2ad()%$$

$head See Also$$
$cref mul_level$$

$head Base$$
This is the base type used to recorded the operation sequence in $icode f$$
and $icode af$$; i.e., the type $codei%AD<%Base%>%$$ was used to record
the operation sequence.

$head f$$
This object has prototype
$codei%
    ADFun<%Base%> %f%
%$$
It does it's derivative calculations using the type $icode Base$$.

$head af$$
This object has prototype
$codei%
    ADFun< AD<%Base%> , %Base% > %af%
%$$
It has the same operation sequence as $icode f$$,
but it does it's derivative calculations using the type
$codei%AD<%Base>%$$.
This enables one to record new functions that are defined
using derivatives of the function $icode f$$.
Initially, there are no Taylor coefficients stored in $icode af$$ and
$cref%af.size_order()%size_order%$$ is zero.

$children%
    example/general/base2ad.cpp
%$$
$head Example$$
The file $cref base2ad.cpp$$
contains an example and test of this operation.

$end
----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file base2ad.hpp
*/
/// Create an ADFun< AD<Base>, Base > from this ADFun<Base>
template <class Base, class RecBase>
ADFun< AD<Base>, RecBase > ADFun<Base,RecBase>::base2ad(void) const
{   ADFun< AD<Base>, RecBase > fun;
    //
    // bool values
    fun.has_been_optimized_        = has_been_optimized_;
    fun.check_for_nan_             = check_for_nan_;
    //
    // size_t values
    fun.compare_change_count_      = compare_change_count_;
    fun.compare_change_number_     = compare_change_number_;
    fun.compare_change_op_index_   = compare_change_op_index_;
    CPPAD_ASSERT_UNKNOWN( fun.num_order_taylor_ == 0 ) ;
    CPPAD_ASSERT_UNKNOWN( fun.cap_order_taylor_ == 0 );
    CPPAD_ASSERT_UNKNOWN( fun.num_direction_taylor_ == 0 );
    fun.num_var_tape_              = num_var_tape_;
    //
    // pod_vector objects
    fun.ind_taddr_                 = ind_taddr_;
    fun.dep_taddr_                 = dep_taddr_;
    fun.dep_parameter_             = dep_parameter_;
    fun.cskip_op_                  = cskip_op_;
    fun.load_op2var_               = load_op2var_;
    //
    // pod_maybe_vector< AD<Base> > = pod_maybe_vector<Base>
    CPPAD_ASSERT_UNKNOWN( fun.taylor_.size() == 0 );
    //
    // player
    // (uses move semantics)
    fun.play_ = play_.base2ad();
    //
    // subgraph
    fun.subgraph_info_ = subgraph_info_;
    //
    // sparse_pack
    fun.for_jac_sparse_pack_ = for_jac_sparse_pack_;
    //
    // sparse_list
    fun.for_jac_sparse_set_  = for_jac_sparse_set_;
    //
    return fun;
}

} // END_CPPAD_NAMESPACE
# endif
