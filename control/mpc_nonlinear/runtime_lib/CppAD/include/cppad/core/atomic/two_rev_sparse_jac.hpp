# ifndef CPPAD_CORE_ATOMIC_TWO_REV_SPARSE_JAC_HPP
# define CPPAD_CORE_ATOMIC_TWO_REV_SPARSE_JAC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_two_rev_sparse_jac$$
$spell
    sq
    mul.hpp
    rt
    afun
    Jacobian
    jac
    CppAD
    std
    bool
    const
    hes
$$

$section Atomic Reverse Jacobian Sparsity Patterns$$

$head Syntax$$
$icode%ok% = %afun%.rev_sparse_jac(%q%, %rt%, %st%, %x%)
%$$

$head Deprecated 2016-06-27$$
$icode%ok% = %afun%.rev_sparse_jac(%q%, %rt%, %st%)
%$$

$head Purpose$$
This function is used by
$cref RevSparseJac$$ to compute
Jacobian sparsity patterns.
If you are using $cref RevSparseJac$$,
one of the versions of this
virtual function must be defined by the
$cref/atomic_user/atomic_two_ctor/atomic_user/$$ class.
$pre

$$
For a fixed matrix $latex R \in \B{R}^{q \times m}$$,
the Jacobian of $latex R * f( x )$$ with respect to $latex x \in \B{R}^n$$ is
$latex \[
    S(x) = R * f^{(1)} (x)
\] $$
Given a $cref/sparsity pattern/glossary/Sparsity Pattern/$$ for $latex R$$,
$code rev_sparse_jac$$ computes a sparsity pattern for $latex S(x)$$.

$head Implementation$$
If you are using
$cref RevSparseJac$$ or $cref ForSparseHes$$,
this virtual function must be defined by the
$cref/atomic_user/atomic_two_ctor/atomic_user/$$ class.

$subhead q$$
The argument $icode q$$ has prototype
$codei%
    size_t %q%
%$$
It specifies the number of rows in
$latex R \in \B{R}^{q \times m}$$ and the Jacobian
$latex S(x) \in \B{R}^{q \times n}$$.

$subhead rt$$
This argument has prototype
$codei%
     const %atomic_sparsity%& %rt%
%$$
and is a
$cref/atomic_sparsity/atomic_two_option/atomic_sparsity/$$ pattern for
$latex R^\R{T} \in \B{R}^{m \times q}$$.

$subhead st$$
This argument has prototype
$codei%
    %atomic_sparsity%& %st%
%$$
The input value of its elements
are not specified (must not matter).
Upon return, $icode s$$ is a
$cref/atomic_sparsity/atomic_two_option/atomic_sparsity/$$ pattern for
$latex S(x)^\R{T} \in \B{R}^{n \times q}$$.

$subhead x$$
$index deprecated$$
The argument has prototype
$codei%
    const CppAD::vector<%Base%>& %x%
%$$
and size is equal to the $icode n$$.
This is the $cref Value$$ corresponding to the parameters in the
vector $cref/ax/atomic_two_afun/ax/$$ (when the atomic function was called).
To be specific, if
$codei%
    if( Parameter(%ax%[%i%]) == true )
        %x%[%i%] = Value( %ax%[%i%] );
    else
        %x%[%i%] = CppAD::numeric_limits<%Base%>::quiet_NaN();
%$$
The version of this function with out the $icode x$$ argument is deprecated;
i.e., you should include the argument even if you do not use it.

$head ok$$
The return value $icode ok$$ has prototype
$codei%
    bool %ok%
%$$
If it is $code true$$, the corresponding evaluation succeeded,
otherwise it failed.

$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/two_rev_sparse_jac.hpp
Atomic reverse mode Jacobian sparsity patterns.
*/
/*!
Link, after case split, from rev_jac_sweep to atomic_base

\param q [in]
is the row dimension for the Jacobian sparsity partterns

\param rt [out]
is the tansposed Jacobian sparsity pattern w.r.t to range variables y

\param st [in]
is the tansposed Jacobian sparsity pattern for the argument variables x

\param x
is the integer value for x arguments that are parameters.
*/
template <class Base>
bool atomic_base<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vector< std::set<size_t> >&       rt ,
          vector< std::set<size_t> >&       st ,
    const vector<Base>&                     x  )
{   return false; }
template <class Base>
bool atomic_base<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vector<bool>&                     rt ,
          vector<bool>&                     st ,
    const vector<Base>&                     x  )
{   return false; }
template <class Base>
bool atomic_base<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vectorBool&                       rt ,
          vectorBool&                       st ,
    const vector<Base>&                     x  )
{   return false; }
// deprecated versions
template <class Base>
bool atomic_base<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vector< std::set<size_t> >&       rt ,
          vector< std::set<size_t> >&       st )
{   return false; }
template <class Base>
bool atomic_base<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vector<bool>&                     rt ,
          vector<bool>&                     st )
{   return false; }
template <class Base>
bool atomic_base<Base>::rev_sparse_jac(
    size_t                                  q  ,
    const vectorBool&                       rt ,
          vectorBool&                       st )
{   return false; }

/*!
Link, before case split, from rev_jac_sweep to atomic_base.

\tparam InternalSparsity
Is the used internaly for sparsity calculations; i.e.,
sparse_pack or sparse_list.

\param x
is parameter arguments to the function, other components are nan.

\param x_index
is the variable index, on the tape, for the arguments to this function.
This size of x_index is n, the number of arguments to this function.

\param y_index
is the variable index, on the tape, for the results for this function.
This size of y_index is m, the number of results for this function.

\param var_sparsity
On input, for i = 0, ... , m-1, the sparsity pattern with index y_index[i],
is the sparsity for the i-th argument to this atomic function.
On output, for j = 0, ... , n-1, the sparsity pattern with index x_index[j],
the sparsity has been updated to remove y as a function of x.
*/
template <class Base>
template <class InternalSparsity>
bool atomic_base<Base>::rev_sparse_jac(
    const vector<Base>&              x            ,
    const local::pod_vector<size_t>& x_index      ,
    const local::pod_vector<size_t>& y_index      ,
    InternalSparsity&                var_sparsity )
{
    // initial results may be non-empty during reverse mode
    size_t q           = var_sparsity.end();
    bool   input_empty = false;
    bool   zero_empty  = true;
    bool   transpose   = false;
    size_t n           = x_index.size();
    bool   ok          = false;
    size_t thread      = thread_alloc::thread_num();
    allocate_work(thread);
    //
    std::string msg    = ": atomic_base.rev_sparse_jac: returned false";
    if( sparsity_ == pack_sparsity_enum )
    {   vectorBool& pack_rt ( work_[thread]->pack_r );
        vectorBool& pack_st ( work_[thread]->pack_s );
        local::sparse::get_internal_pattern(
            transpose, y_index, var_sparsity, pack_rt
        );
        //
        pack_st.resize(n * q );
        ok = rev_sparse_jac(q, pack_rt, pack_st, x);
        if( ! ok )
            ok = rev_sparse_jac(q, pack_rt, pack_st);
        if( ! ok )
        {   msg = atomic_name() + msg + " sparsity = pack_sparsity_enum";
            CPPAD_ASSERT_KNOWN(false, msg.c_str());
        }
        local::sparse::set_internal_pattern(zero_empty, input_empty,
            transpose, x_index, var_sparsity, pack_st
        );
    }
    else if( sparsity_ == bool_sparsity_enum )
    {   vector<bool>& bool_rt ( work_[thread]->bool_r );
        vector<bool>& bool_st ( work_[thread]->bool_s );
        local::sparse::get_internal_pattern(
            transpose, y_index, var_sparsity, bool_rt
        );
        bool_st.resize(n * q );
        ok = rev_sparse_jac(q, bool_rt, bool_st, x);
        if( ! ok )
            ok = rev_sparse_jac(q, bool_rt, bool_st);
        if( ! ok )
        {   msg = atomic_name() + msg + " sparsity = bool_sparsity_enum";
            CPPAD_ASSERT_KNOWN(false, msg.c_str());
        }
        local::sparse::set_internal_pattern(zero_empty, input_empty,
            transpose, x_index, var_sparsity, bool_st
        );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( sparsity_ == set_sparsity_enum );
        vector< std::set<size_t> >& set_rt ( work_[thread]->set_r );
        vector< std::set<size_t> >& set_st ( work_[thread]->set_s );
        local::sparse::get_internal_pattern(
            transpose, y_index, var_sparsity, set_rt
        );
        set_st.resize(n);
        ok = rev_sparse_jac(q, set_rt, set_st, x);
        if( ! ok )
            ok = rev_sparse_jac(q, set_rt, set_st);
        if( ! ok )
        {   msg = atomic_name() + msg + " sparsity = set_sparsity_enum";
            CPPAD_ASSERT_KNOWN(false, msg.c_str());
        }
        local::sparse::set_internal_pattern(zero_empty, input_empty,
            transpose, x_index, var_sparsity, set_st
        );
    }
    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
