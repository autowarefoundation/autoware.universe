# ifndef CPPAD_CORE_CHKPOINT_TWO_DYNAMIC_HPP
# define CPPAD_CORE_CHKPOINT_TWO_DYNAMIC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin chkpoint_two_dynamic$$
$spell
    chk
    chkpoint
    dyn_ind
$$

$section Dynamic Parameters in Checkpoint Functions$$

$head Syntax$$
$icode%chk_fun%.new_dynamic(%dynamic%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head chk_fun$$
This object must have been created using the
$cref/chkpoint_two/chkpoint_two_ctor/chk_fun/$$ constructor.

$subhead Base$$
This is the $cref/Base/chkpoint_two_ctor/Base/$$ type
in the $icode chk_fun$$ constructor.

$subhead fun$$
This is the function $cref/fun/chkpoint_two_ctor/fun/$$
in the $icode chk_fun$$ constructor.

$head BaseVector$$
This must be a $cref SimpleVector$$ with elements of type $icode Base$$.

$head dynamic$$
This is a vector with new values for the dynamic parameters
in the function $icode fun$$.
Is size must be equal to
$cref/fun.size_dyn_ind()/fun_property/size_dyn_par/$$.
This only affects the copy of $icode fun$$ used by $icode chk_fun$$.

$head Multi-Threading$$
If one is using $cref/in_parallel/ta_in_parallel/$$,
there is a separate copy of $icode fun$$ for each thread.
In this case, only the dynamic parameters in the copy for the current
$cref/thread number/ta_thread_num/$$ are changed.


$end
*/
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file chkpoint_two/dynamic.hpp
Change the dynnamic parameter in a checkpoint function.
*/

/*!
Constructor

\tparam Base
base class for recording AD<Base> operations using this checkpoint object.

\param dynamic
is the new values for the dynamic parameters in the function
defining this checkpoint object.
*/

// BEGIN_PROTOTYPE
template <class Base>
template <class BaseVector>
void chkpoint_two<Base>::new_dynamic(const BaseVector& dynamic)
// END_PROTOTYPE
{   ADFun<Base>* g_ptr = &g_;
    if( use_in_parallel_ )
    {   size_t thread = thread_alloc::thread_num();
        allocate_member(thread);
        g_ptr = &(member_[thread]->g_);
    }
# ifndef NDEBUG
    else if( thread_alloc::in_parallel() )
    {   std::string msg = atomic_three<Base>::atomic_name();
        msg += ": use_in_parallel is false and in_parallel() is true";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# endif
    g_ptr->new_dynamic(dynamic);
}

} // END_CPPAD_NAMESPACE
# endif
