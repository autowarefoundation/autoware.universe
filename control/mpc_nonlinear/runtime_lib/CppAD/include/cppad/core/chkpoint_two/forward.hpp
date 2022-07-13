# ifndef CPPAD_CORE_CHKPOINT_TWO_FORWARD_HPP
# define CPPAD_CORE_CHKPOINT_TWO_FORWARD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file chkpoint_two/forward.hpp
Second generation checkpoint forward mode.
*/
/*!
Link from chkpoint_two to forward mode

\param parameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param need_y [in]
specifies which components of taylor_y are needed,

\param order_low [in]
lowerest order for this forward mode calculation.

\param order_up [in]
highest order for this forward mode calculation.

\param taylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param taylor_y [out]
Taylor coefficient corresponding to y for this calculation

See the forward mode in user's documentation for atomic_three
*/
template <class Base>
bool chkpoint_two<Base>::forward(
    const vector<Base>&          parameter_x ,
    const vector<ad_type_enum>&  type_x      ,
    size_t                       need_y      ,
    size_t                       order_low   ,
    size_t                       order_up    ,
    const vector<Base>&          taylor_x    ,
    vector<Base>&                taylor_y    )
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
    // compute forward mode results for all values and orders
    taylor_y = g_ptr->Forward(order_up, taylor_x);
    //
    return true;
}
/*!
Link from chkpoint_two to AD forward mode

\param aparameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param need_y [in]
specifies which components of taylor_y are needed,

\param order_low [in]
lowerest order for this forward mode calculation.

\param order_up [in]
highest order for this forward mode calculation.

\param ataylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param ataylor_y [out]
Taylor coefficient corresponding to y for this calculation

See the forward mode in user's documentation for atomic_three
*/
template <class Base>
bool chkpoint_two<Base>::forward(
    const vector< AD<Base> >&    aparameter_x ,
    const vector<ad_type_enum>&  type_x       ,
    size_t                       need_y       ,
    size_t                       order_low    ,
    size_t                       order_up     ,
    const vector< AD<Base> >&    ataylor_x    ,
    vector< AD<Base> >&          ataylor_y    )
{   if( ! use_base2ad_ )
        return false;
    //
    ADFun< AD<Base>, Base >* ag_ptr = &ag_;
    if( use_in_parallel_ )
    {   size_t thread = thread_alloc::thread_num();
        allocate_member(thread);
        ag_ptr = &(member_[thread]->ag_);
    }
# ifndef NDEBUG
    else if( thread_alloc::in_parallel() )
    {   std::string msg = atomic_three<Base>::atomic_name();
        msg += ": use_in_parallel is false and in_parallel() is true";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# endif
    // compute forward mode results for all values and orders
    ataylor_y = ag_ptr->Forward(order_up, ataylor_x);
    //
    return true;
}

} // END_CPPAD_NAMESPACE
# endif
