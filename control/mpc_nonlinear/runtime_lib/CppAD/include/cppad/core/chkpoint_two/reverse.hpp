# ifndef CPPAD_CORE_CHKPOINT_TWO_REVERSE_HPP
# define CPPAD_CORE_CHKPOINT_TWO_REVERSE_HPP
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
\file chkpoint_two/reverse.hpp
Second generation checkpoint reverse mode.
*/
/*!
Link from chkpoint_two to reverse mode

\param parameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param order_up [in]
highest order Taylor coefficient aht we are computing derivative of

\param taylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param taylor_y [in]
Taylor coefficient corresponding to y for this calculation

\param partial_x [out]
Partials w.r.t. the x Taylor coefficients.

\param partial_y [in]
Partials w.r.t. the y Taylor coefficients.

See the reverse mode in user's documentation for atomic_three
*/
template <class Base>
bool chkpoint_two<Base>::reverse(
    const vector<Base>&         parameter_x   ,
    const vector<ad_type_enum>& type_x        ,
    size_t                      order_up      ,
    const vector<Base>&         taylor_x      ,
    const vector<Base>&         taylor_y      ,
    vector<Base>&               partial_x     ,
    const vector<Base>&         partial_y     )

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
    // compute forward mode Taylor coefficient orders 0 through order_up
# ifdef NDEBUG
    g_ptr->Forward(order_up, taylor_x);
# else
    vector<Base> check = g_ptr->Forward(order_up, taylor_x);
    CPPAD_ASSERT_UNKNOWN( taylor_y.size() == check.size() )
    for(size_t i = 0; i < taylor_y.size(); ++i)
        CPPAD_ASSERT_UNKNOWN( taylor_y[i] == check[i] );
# endif
    // now can run reverse mode
    partial_x = g_ptr->Reverse(order_up+1, partial_y);
    //
    return true;
}
/*!
Link from chkpoint_two to AD reverse mode

\param aparameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param order_up [in]
highest order Taylor coefficient aht we are computing derivative of

\param ataylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param ataylor_y [in]
Taylor coefficient corresponding to y for this calculation

\param apartial_x [out]
Partials w.r.t. the x Taylor coefficients.

\param apartial_y [in]
Partials w.r.t. the y Taylor coefficients.

See the reverse mode in user's documentation for atomic_three
*/
template <class Base>
bool chkpoint_two<Base>::reverse(
    const vector< AD<Base> >&   aparameter_x ,
    const vector<ad_type_enum>& type_x       ,
    size_t                      order_up     ,
    const vector< AD<Base> >&   ataylor_x    ,
    const vector< AD<Base> >&   ataylor_y    ,
    vector< AD<Base> >&         apartial_x   ,
    const vector< AD<Base> >&   apartial_y   )
{   ADFun< AD<Base>, Base >* ag_ptr = &ag_;
    if( use_in_parallel_ )
    {   size_t thread = thread_alloc::thread_num();
        allocate_member(thread);
        ag_ptr = &(member_[thread]->ag_);
    }
    // compute forward mode Taylor coefficient orders 0 through order_up
# ifdef NDEBUG
    ag_ptr->Forward(order_up, ataylor_x);
# else
    vector< AD<Base> > acheck = ag_ptr->Forward(order_up, ataylor_x);
    CPPAD_ASSERT_UNKNOWN( ataylor_y.size() == acheck.size() )
    for(size_t i = 0; i < ataylor_y.size(); ++i)
        CPPAD_ASSERT_UNKNOWN( ataylor_y[i] == acheck[i] );
# endif
    // now can run reverse mode
    apartial_x = ag_ptr->Reverse(order_up+1, apartial_y);
    //
    return true;
}

} // END_CPPAD_NAMESPACE
# endif
