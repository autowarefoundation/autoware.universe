# ifndef CPPAD_CORE_ATOMIC_TWO_FOR_SPARSE_JAC_HPP
# define CPPAD_CORE_ATOMIC_TWO_FOR_SPARSE_JAC_HPP
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
$begin atomic_two_for_sparse_jac$$
$spell
    sq
    mul.hpp
    afun
    Jacobian
    jac
    const
    CppAD
    std
    bool
    std
$$

$section Atomic Forward Jacobian Sparsity Patterns$$

$head Syntax$$
$icode%ok% = %afun%.for_sparse_jac(%q%, %r%, %s%, %x%)
%$$

$head Deprecated 2016-06-27$$
$icode%ok% = %afun%.for_sparse_jac(%q%, %r%, %s%)
%$$

$head Purpose$$
This function is used by $cref ForSparseJac$$ to compute
Jacobian sparsity patterns.
For a fixed matrix $latex R \in \B{R}^{n \times q}$$,
the Jacobian of $latex f( x + R * u)$$ with respect to $latex u \in \B{R}^q$$ is
$latex \[
    S(x) = f^{(1)} (x) * R
\] $$
Given a $cref/sparsity pattern/glossary/Sparsity Pattern/$$ for $latex R$$,
$code for_sparse_jac$$ computes a sparsity pattern for $latex S(x)$$.

$head Implementation$$
If you are using
$cref ForSparseJac$$,
$cref ForSparseHes$$, or
$cref RevSparseHes$$,
one of the versions of this
virtual function must be defined by the
$cref/atomic_user/atomic_two_ctor/atomic_user/$$ class.

$subhead q$$
The argument $icode q$$ has prototype
$codei%
    size_t %q%
%$$
It specifies the number of columns in
$latex R \in \B{R}^{n \times q}$$ and the Jacobian
$latex S(x) \in \B{R}^{m \times q}$$.

$subhead r$$
This argument has prototype
$codei%
     const %atomic_sparsity%& %r%
%$$
and is a $cref/atomic_sparsity/atomic_two_option/atomic_sparsity/$$ pattern for
$latex R \in \B{R}^{n \times q}$$.

$subhead s$$
This argument has prototype
$codei%
    %atomic_sparsity%& %s%
%$$
The input values of its elements
are not specified (must not matter).
Upon return, $icode s$$ is a
$cref/atomic_sparsity/atomic_two_option/atomic_sparsity/$$ pattern for
$latex S(x) \in \B{R}^{m \times q}$$.

$subhead x$$
$index deprecated$$
The argument has prototype
$codei%
    const CppAD::vector<%Base%>& %x%
%$$
and size is equal to the $icode n$$.
This is the $cref Value$$ value corresponding to the parameters in the
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
\file atomic/two_for_sparse_jac.hpp
Atomic forward Jacobian sparsity pattern.
*/
/*!
Link, after case split, from for_jac_sweep to atomic_base.

\param q
is the column dimension for the Jacobian sparsity partterns.

\param r
is the Jacobian sparsity pattern for the argument vector x

\param s
is the Jacobian sparsity pattern for the result vector y

\param x
is the integer value for x arguments that are parameters.
*/
template <class Base>
bool atomic_base<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vector< std::set<size_t> >&       r  ,
          vector< std::set<size_t> >&       s  ,
    const vector<Base>&                     x  )
{   return false; }
template <class Base>
bool atomic_base<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vector<bool>&                     r  ,
          vector<bool>&                     s  ,
    const vector<Base>&                     x  )
{   return false; }
template <class Base>
bool atomic_base<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vectorBool&                       r  ,
          vectorBool&                       s  ,
    const vector<Base>&                     x  )
{   return false; }
// deprecated versions
template <class Base>
bool atomic_base<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vector< std::set<size_t> >&       r  ,
          vector< std::set<size_t> >&       s  )
{   return false; }
template <class Base>
bool atomic_base<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vector<bool>&                     r  ,
          vector<bool>&                     s  )
{   return false; }
template <class Base>
bool atomic_base<Base>::for_sparse_jac(
    size_t                                  q  ,
    const vectorBool&                       r  ,
          vectorBool&                       s  )
{   return false; }

/*!
Link, before case split, from for_jac_sweep to atomic_base.

\tparam InternalSparsity
Is the type used for internal sparsity calculations; i.e.,
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
On input, for j = 0, ... , n-1, the sparsity pattern with index x_index[j],
is the sparsity for the j-th argument to this atomic function.
On output, for i = 0, ... , m-1, the sparsity pattern with index y_index[i],
is the sparsity for the i-th result for this atomic function.
*/
template <class Base>
template <class InternalSparsity>
bool atomic_base<Base>::for_sparse_jac(
    const vector<Base>&              x            ,
    const local::pod_vector<size_t>& x_index      ,
    const local::pod_vector<size_t>& y_index      ,
    InternalSparsity&                var_sparsity )
{
    // intial results are empty during forward mode
    size_t q           = var_sparsity.end();
    bool   input_empty = true;
    bool   zero_empty  = true;
    bool   transpose   = false;
    size_t m           = y_index.size();
    bool   ok          = false;
    size_t thread      = thread_alloc::thread_num();
    allocate_work(thread);
    //
    std::string msg    = ": atomic_base.for_sparse_jac: returned false";
    if( sparsity_ == pack_sparsity_enum )
    {   vectorBool& pack_r ( work_[thread]->pack_r );
        vectorBool& pack_s ( work_[thread]->pack_s );
        local::sparse::get_internal_pattern(
            transpose, x_index, var_sparsity, pack_r
        );
        //
        pack_s.resize(m * q );
        ok = for_sparse_jac(q, pack_r, pack_s, x);
        if( ! ok )
            ok = for_sparse_jac(q, pack_r, pack_s);
        if( ! ok )
        {   msg = atomic_name() + msg + " sparsity = pack_sparsity_enum";
            CPPAD_ASSERT_KNOWN(false, msg.c_str());
        }
        local::sparse::set_internal_pattern(zero_empty, input_empty,
            transpose, y_index, var_sparsity, pack_s
        );
    }
    else if( sparsity_ == bool_sparsity_enum )
    {   vector<bool>& bool_r ( work_[thread]->bool_r );
        vector<bool>& bool_s ( work_[thread]->bool_s );
        local::sparse::get_internal_pattern(
            transpose, x_index, var_sparsity, bool_r
        );
        bool_s.resize(m * q );
        ok = for_sparse_jac(q, bool_r, bool_s, x);
        if( ! ok )
            ok = for_sparse_jac(q, bool_r, bool_s);
        if( ! ok )
        {   msg = atomic_name() + msg + " sparsity = bool_sparsity_enum";
            CPPAD_ASSERT_KNOWN(false, msg.c_str());
        }
        local::sparse::set_internal_pattern(zero_empty, input_empty,
            transpose, y_index, var_sparsity, bool_s
        );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( sparsity_ == set_sparsity_enum );
        vector< std::set<size_t> >& set_r ( work_[thread]->set_r );
        vector< std::set<size_t> >& set_s ( work_[thread]->set_s );
        local::sparse::get_internal_pattern(
            transpose, x_index, var_sparsity, set_r
        );
        //
        set_s.resize(m);
        ok = for_sparse_jac(q, set_r, set_s, x);
        if( ! ok )
            ok = for_sparse_jac(q, set_r, set_s);
        if( ! ok )
        {   msg = atomic_name() + msg + " sparsity = set_sparsity_enum";
            CPPAD_ASSERT_KNOWN(false, msg.c_str());
        }
        local::sparse::set_internal_pattern(zero_empty, input_empty,
            transpose, y_index, var_sparsity, set_s
        );
    }
    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
