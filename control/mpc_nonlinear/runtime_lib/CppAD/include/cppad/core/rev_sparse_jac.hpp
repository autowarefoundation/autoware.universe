# ifndef CPPAD_CORE_REV_SPARSE_JAC_HPP
# define CPPAD_CORE_REV_SPARSE_JAC_HPP
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
$begin RevSparseJac$$
$spell
    optimizer
    nz
    CondExpRel
    std
    VecAD
    var
    Jacobian
    Jac
    const
    Bool
    Dep
    proportional
$$

$section Jacobian Sparsity Pattern: Reverse Mode$$

$head Syntax$$
$icode%s% = %f%.RevSparseJac(%q%, %r%)
%$$
$icode%s% = %f%.RevSparseJac(%q%, %r%, %transpose%, %dependency%)%$$

$head Purpose$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to $icode f$$.
For a fixed matrix $latex R \in \B{R}^{q \times m}$$,
the Jacobian of $latex R * F( x )$$
with respect to $latex x$$ is
$latex \[
    S(x) = R * F^{(1)} ( x )
\] $$
Given a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for $latex R$$,
$code RevSparseJac$$ returns a sparsity pattern for the $latex S(x)$$.

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$

$head x$$
If the operation sequence in $icode f$$ is
$cref/independent/glossary/Operation/Independent/$$ of
the independent variables in $latex x \in \B{R}^n$$,
the sparsity pattern is valid for all values of
(even if it has $cref CondExp$$ or $cref VecAD$$ operations).

$head q$$
The argument $icode q$$ has prototype
$codei%
    size_t %q%
%$$
It specifies the number of rows in
$latex R \in \B{R}^{q \times m}$$ and the
Jacobian $latex S(x) \in \B{R}^{q \times n}$$.

$head transpose$$
The argument $icode transpose$$ has prototype
$codei%
    bool %transpose%
%$$
The default value $code false$$ is used when $icode transpose$$ is not present.

$head dependency$$
The argument $icode dependency$$ has prototype
$codei%
    bool %dependency%
%$$
If $icode dependency$$ is true,
the $cref/dependency pattern/dependency.cpp/Dependency Pattern/$$
(instead of sparsity pattern) is computed.

$head r$$
The argument $icode s$$ has prototype
$codei%
    const %SetVector%& %r%
%$$
see $cref/SetVector/RevSparseJac/SetVector/$$ below.

$subhead transpose false$$
If $icode r$$ has elements of type $code bool$$,
its size is $latex q * m$$.
If it has elements of type $code std::set<size_t>$$,
its size is $icode q$$ and all its set elements are between
zero and $latex m - 1$$.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex R \in \B{R}^{q \times m}$$.

$subhead transpose true$$
If $icode r$$ has elements of type $code bool$$,
its size is $latex m * q$$.
If it has elements of type $code std::set<size_t>$$,
its size is $icode m$$ and all its set elements are between
zero and $latex q - 1$$.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex R^\R{T} \in \B{R}^{m \times q}$$.

$head s$$
The return value $icode s$$ has prototype
$codei%
    %SetVector% %s%
%$$
see $cref/SetVector/RevSparseJac/SetVector/$$ below.

$subhead transpose false$$
If it has elements of type $code bool$$,
its size is $latex q * n$$.
If it has elements of type $code std::set<size_t>$$,
its size is $icode q$$ and all its set elements are between
zero and $latex n - 1$$.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex S(x) \in {q \times n}$$.

$subhead transpose true$$
If it has elements of type $code bool$$,
its size is $latex n * q$$.
If it has elements of type $code std::set<size_t>$$,
its size is $icode n$$ and all its set elements are between
zero and $latex q - 1$$.
It specifies a
$cref/sparsity pattern/glossary/Sparsity Pattern/$$
for the matrix $latex S(x)^\R{T} \in {n \times q}$$.

$head SetVector$$
The type $icode SetVector$$ must be a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$code bool$$ or $code std::set<size_t>$$;
see $cref/sparsity pattern/glossary/Sparsity Pattern/$$ for a discussion
of the difference.

$head Entire Sparsity Pattern$$
Suppose that $latex q = m$$ and
$latex R$$ is the $latex m \times m$$ identity matrix.
In this case,
the corresponding value for $icode s$$ is a
sparsity pattern for the Jacobian $latex S(x) = F^{(1)} ( x )$$.

$head Example$$
$children%
    example/sparse/rev_sparse_jac.cpp
%$$
The file
$cref rev_sparse_jac.cpp$$
contains an example and test of this operation.

$end
-----------------------------------------------------------------------------
*/

# include <cppad/local/std_set.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file core/rev_sparse_jac.hpp
Reverse mode Jacobian sparsity patterns.
*/
// =========================================================================
// RevSparseJacCase

/*!
Private helper function for RevSparseJac(q, r, transpose) boolean sparsity.

All of the description in the public member function
 RevSparseJac(q, r, transpose) apply.

\param set_type
is a bool value.
This argument is used to dispatch to the proper source code
depending on the value of SetVector::value_type.

\param transpose
See RevSparseJac(q, r, transpose, dependency)

\param dependency
See RevSparseJac(q, r, transpose, dependency)

\param q
See RevSparseJac(q, r, transpose, dependency)

\param r
See RevSparseJac(q, r, transpose, dependency)

\param s
is the return value for the corresponding call to
RevSparseJac(q, r, transpose).
*/

template <class Base, class RecBase>
template <class SetVector>
void ADFun<Base,RecBase>::RevSparseJacCase(
    bool                set_type          ,
    bool                transpose         ,
    bool                dependency        ,
    size_t              q                 ,
    const SetVector&    r                 ,
    SetVector&          s                 )
{
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);
    //
    size_t n = Domain();
    size_t m = Range();

    // dimension of the result vector
    s.resize( q * n );

    // check SetVector is Simple Vector class with bool elements
    CheckSimpleVector<bool, SetVector>();
    //
    CPPAD_ASSERT_KNOWN(
        q > 0,
        "RevSparseJac: q is not greater than zero"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(r.size()) == q * m,
        "RevSparseJac: size of r is not equal to\n"
        "q times range dimension for ADFun object."
    );
    //
    // vector of sets that will hold the results
    local::sparse::pack_setvec    var_sparsity;
    var_sparsity.resize(num_var_tape_, q);

    // The sparsity pattern corresponding to the dependent variables
    for(size_t i = 0; i < m; i++)
    {   CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ );
        if( transpose )
        {   for(size_t j = 0; j < q; j++) if( r[ i * q + j ] )
                var_sparsity.post_element( dep_taddr_[i], j );
        }
        else
        {   for(size_t j = 0; j < q; j++) if( r[ j * m + i ] )
                var_sparsity.post_element( dep_taddr_[i], j );
        }
    }
    // process posts
    for(size_t i = 0; i < m; i++)
        var_sparsity.process_post( dep_taddr_[i] );

    // evaluate the sparsity patterns
    local::sweep::rev_jac<addr_t>(
        &play_,
        dependency,
        n,
        num_var_tape_,
        var_sparsity,
        not_used_rec_base

    );

    // return values corresponding to dependent variables
    CPPAD_ASSERT_UNKNOWN( size_t(s.size()) == q * n );
    for(size_t j = 0; j < n; j++)
    {   CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] == (j+1) );

        // ind_taddr_[j] is operator taddr for j-th independent variable
        CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == local::InvOp );

        // extract the result from var_sparsity
        if( transpose )
        {   for(size_t i = 0; i < q; i++)
                s[ j * q + i ] = false;
        }
        else
        {   for(size_t i = 0; i < q; i++)
                s[ i * n + j ] = false;
        }
        CPPAD_ASSERT_UNKNOWN( var_sparsity.end() == q );
        local::sparse::pack_setvec::const_iterator itr(var_sparsity, j+1);
        size_t i = *itr;
        while( i < q )
        {   if( transpose )
                s[ j * q + i ] = true;
            else
                s[ i * n + j ] = true;
            i  = *(++itr);
        }
    }
}

/*!
Private helper function for RevSparseJac(q, r, transpose) set sparsity

All of the description in the public member function
 RevSparseJac(q, r, transpose) apply.

\param set_type
is a std::set<size_t> object.
This argument is used to dispatch to the proper source code
depending on the value of SetVector::value_type.

\param transpose
See RevSparseJac(q, r, transpose, dependency)

\param dependency
See RevSparseJac(q, r, transpose, dependency)

\param q
See RevSparseJac(q, r, transpose, dependency)

\param r
See RevSparseJac(q, r, transpose, dependency)

\param s
is the return value for the corresponding call to RevSparseJac(q, r, transpose)
*/

template <class Base, class RecBase>
template <class SetVector>
void ADFun<Base,RecBase>::RevSparseJacCase(
    const std::set<size_t>&      set_type          ,
    bool                         transpose         ,
    bool                         dependency        ,
    size_t                       q                 ,
    const SetVector&             r                 ,
    SetVector&                   s                 )
{
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);
    //
    // dimension of the result vector
    if( transpose )
        s.resize( Domain() );
    else
        s.resize( q );

    // temporary indices
    std::set<size_t>::const_iterator itr_1;

    // check SetVector is Simple Vector class with sets for elements
    CheckSimpleVector<std::set<size_t>, SetVector>(
        local::one_element_std_set<size_t>(), local::two_element_std_set<size_t>()
    );

    // domain dimensions for F
    size_t n = ind_taddr_.size();
    size_t m = dep_taddr_.size();

    CPPAD_ASSERT_KNOWN(
        q > 0,
        "RevSparseJac: q is not greater than zero"
    );
    CPPAD_ASSERT_KNOWN(
        size_t(r.size()) == q || transpose,
        "RevSparseJac: size of r is not equal to q and transpose is false."
    );
    CPPAD_ASSERT_KNOWN(
        size_t(r.size()) == m || ! transpose,
        "RevSparseJac: size of r is not equal to m and transpose is true."
    );

    // vector of lists that will hold the results
    local::sparse::list_setvec   var_sparsity;
    var_sparsity.resize(num_var_tape_, q);

    // The sparsity pattern corresponding to the dependent variables
    if( transpose )
    {   for(size_t i = 0; i < m; i++)
        {   itr_1 = r[i].begin();
            while(itr_1 != r[i].end())
            {   size_t j = *itr_1++;
                CPPAD_ASSERT_KNOWN(
                j < q,
                "RevSparseJac: transpose is true and element of the set\n"
                "r[i] has value greater than or equal q."
                );
                CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ );
                var_sparsity.post_element( dep_taddr_[i], j );
            }
        }
    }
    else
    {   for(size_t i = 0; i < q; i++)
        {   itr_1 = r[i].begin();
            while(itr_1 != r[i].end())
            {   size_t j = *itr_1++;
                CPPAD_ASSERT_KNOWN(
                j < m,
                "RevSparseJac: transpose is false and element of the set\n"
                "r[i] has value greater than or equal range dimension."
                );
                CPPAD_ASSERT_UNKNOWN( dep_taddr_[j] < num_var_tape_ );
                var_sparsity.post_element( dep_taddr_[j], i );
            }
        }
    }
    // process posts
    for(size_t i = 0; i < m; i++)
        var_sparsity.process_post( dep_taddr_[i] );

    // evaluate the sparsity patterns
    local::sweep::rev_jac<addr_t>(
        &play_,
        dependency,
        n,
        num_var_tape_,
        var_sparsity,
        not_used_rec_base

    );

    // return values corresponding to dependent variables
    CPPAD_ASSERT_UNKNOWN( size_t(s.size()) == q || transpose );
    CPPAD_ASSERT_UNKNOWN( size_t(s.size()) == n || ! transpose );
    for(size_t j = 0; j < n; j++)
    {   CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] == (j+1) );

        // ind_taddr_[j] is operator taddr for j-th independent variable
        CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == local::InvOp );

        CPPAD_ASSERT_UNKNOWN( var_sparsity.end() == q );
        local::sparse::list_setvec::const_iterator itr_2(var_sparsity, j+1);
        size_t i = *itr_2;
        while( i < q )
        {   if( transpose )
                s[j].insert(i);
            else
                s[i].insert(j);
            i = *(++itr_2);
        }
    }
}

// =========================================================================
// RevSparseJac
/*!
User API for Jacobian sparsity patterns using reverse mode.

The C++ source code corresponding to this operation is
\verbatim
    s = f.RevSparseJac(q, r, transpose, dependency)
\endverbatim

\tparam Base
is the base type for this recording.

\tparam SetVector
is a simple vector with elements of type bool.
or std::set<size_t>.

\param q
is the number of rows in the matrix \f$ R \f$.

\param r
is a sparsity pattern for the matrix \f$ R \f$.

\param transpose
are the sparsity patterns for \f$ R \f$ and \f$ S(x) \f$ transposed.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
    CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.


\return
If transpose is false (true), the return value is a sparsity pattern
for \f$ S(x) \f$ (\f$ S(x)^T \f$) where
\f[
    S(x) = R * F^{(1)} (x)
\f]
and \f$ F \f$ is the function corresponding to the operation sequence
and x is any argument value.
If SetVector::value_type is bool,
the return value has size \f$ q * n \f$ ( \f$ n * q \f$).
If SetVector::value_type is std::set<size_t>,
the return value has size \f$ q \f$ ( \f$ n \f$)
and with all its elements between zero and \f$ n - 1 \f$ (\f$ q - 1 \f$).
*/
template <class Base, class RecBase>
template <class SetVector>
SetVector ADFun<Base,RecBase>::RevSparseJac(
    size_t              q          ,
    const SetVector&    r          ,
    bool                transpose  ,
    bool                dependency )
{
    SetVector s;
    typedef typename SetVector::value_type Set_type;

    RevSparseJacCase(
        Set_type()    ,
        transpose     ,
        dependency    ,
        q             ,
        r             ,
        s
    );
    return s;
}
// ===========================================================================
// RevSparseJacCheckpoint
/*!
Reverse mode Jacobian sparsity calculation used by checkpoint functions.

\tparam Base
is the base type for this recording.

\param transpose
is true (false) s is equal to \f$ S(x) \f$ (\f$ S(x)^T \f$)
where
\f[
    S(x) = R * F^{(1)} (x)
\f]
where \f$ F \f$ is the function corresponding to the operation sequence
and \f$ x \f$ is any argument value.

\param q
is the number of rows in the matrix \f$ R \f$.

\param r
is a sparsity pattern for the matrix \f$ R \f$.

\param transpose
are the sparsity patterns for \f$ R \f$ and \f$ S(x) \f$ transposed.

\param dependency
Are the derivatives with respect to left and right of the expression below
considered to be non-zero:
\code
    CondExpRel(left, right, if_true, if_false)
\endcode
This is used by the optimizer to obtain the correct dependency relations.

\param s
The input size and elements of s do not matter.
On output, s is the sparsity pattern for the matrix \f$ S(x) \f$
or \f$ S(x)^T \f$ depending on transpose.

*/
template <class Base, class RecBase>
void ADFun<Base,RecBase>::RevSparseJacCheckpoint(
    size_t                               q          ,
    const local::sparse::list_setvec&    r          ,
    bool                                 transpose  ,
    bool                                 dependency ,
    local::sparse::list_setvec&          s          )
{
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);
    //
    size_t n = Domain();
    size_t m = Range();

# ifndef NDEBUG
    if( transpose )
    {   CPPAD_ASSERT_UNKNOWN( r.n_set() == m );
        CPPAD_ASSERT_UNKNOWN( r.end()   == q );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( r.n_set() == q );
        CPPAD_ASSERT_UNKNOWN( r.end()   == m );
    }
    for(size_t i = 0; i < m; i++)
        CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_ );
# endif

    // holds reverse Jacobian sparsity pattern for all variables
    local::sparse::list_setvec var_sparsity;
    var_sparsity.resize(num_var_tape_, q);

    // set sparsity pattern for dependent variables
    if( transpose )
    {   for(size_t i = 0; i < m; i++)
        {   local::sparse::list_setvec::const_iterator itr(r, i);
            size_t j = *itr;
            while( j < q )
            {   var_sparsity.post_element( dep_taddr_[i], j );
                j = *(++itr);
            }
        }
    }
    else
    {   for(size_t j = 0; j < q; j++)
        {   local::sparse::list_setvec::const_iterator itr(r, j);
            size_t i = *itr;
            while( i < m )
            {   var_sparsity.post_element( dep_taddr_[i], j );
                i = *(++itr);
            }
        }
    }
    // process posts
    for(size_t i = 0; i < m; i++)
        var_sparsity.process_post( dep_taddr_[i] );

    // evaluate the sparsity pattern for all variables
    local::sweep::rev_jac<addr_t>(
        &play_,
        dependency,
        n,
        num_var_tape_,
        var_sparsity,
        not_used_rec_base

    );

    // dimension the return value
    if( transpose )
        s.resize(n, m);
    else
        s.resize(m, n);

    // return values corresponding to independent variables
    for(size_t j = 0; j < n; j++)
    {   CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] == (j+1) );

        // ind_taddr_[j] is operator taddr for j-th independent variable
        CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == local::InvOp );

        // extract the result from var_sparsity
        CPPAD_ASSERT_UNKNOWN( var_sparsity.end() == q );
        local::sparse::list_setvec::const_iterator itr(var_sparsity, j+1);
        size_t i = *itr;
        while( i < q )
        {   if( transpose )
                s.post_element(j, i);
            else
                s.post_element(i, j);
            i  = *(++itr);
        }
    }
    // process posts
    for(size_t i = 0; i < s.n_set(); i++)
        s.process_post(i);

}

} // END_CPPAD_NAMESPACE
# endif
