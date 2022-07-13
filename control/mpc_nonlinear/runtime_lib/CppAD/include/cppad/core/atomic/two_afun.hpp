# ifndef CPPAD_CORE_ATOMIC_TWO_AFUN_HPP
# define CPPAD_CORE_ATOMIC_TWO_AFUN_HPP
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
$begin atomic_two_afun$$

$spell
    sq
    mul
    afun
    const
    CppAD
    mat_mul.cpp
$$

$section Using AD Version of Atomic Function$$

$head Syntax$$
$icode%afun%(%ax%, %ay%)%$$

$head Purpose$$
Given $icode ax$$,
this call computes the corresponding value of $icode ay$$.
If $codei%AD<%Base%>%$$ operations are being recorded,
it enters the computation as an atomic operation in the recording;
see $cref/start recording/Independent/Start Recording/$$.

$head ADVector$$
The type $icode ADVector$$ must be a
$cref/simple vector class/SimpleVector/$$ with elements of type
$codei%AD<%Base%>%$$; see $cref/Base/atomic_two_ctor/atomic_base/Base/$$.

$head afun$$
is a $cref/atomic_user/atomic_two_ctor/atomic_user/$$ object
and this $icode afun$$ function call is implemented by the
$cref/atomic/atomic_two_ctor/atomic_base/$$ class.

$head ax$$
This argument has prototype
$codei%
    const %ADVector%& %ax%
%$$
and size must be equal to $icode n$$.
It specifies vector $latex x \in \B{R}^n$$
at which an $codei%AD<%Base%>%$$ version of
$latex y = f(x)$$ is to be evaluated; see
$cref/Base/atomic_two_ctor/atomic_base/Base/$$.

$head ay$$
This argument has prototype
$codei%
    %ADVector%& %ay%
%$$
and size must be equal to $icode m$$.
The input values of its elements
are not specified (must not matter).
Upon return, it is an $codei%AD<%Base%>%$$ version of
$latex y = f(x)$$.

$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/two_afun.hpp
Implement user call to an atomic_two function.
*/

/*!
Implement the user call to afun(ax, ay) and atomic_one call to
afun(ax, ay, id).

\tparam ADVector
A simple vector class with elements of type <code>AD<Base></code>.

\param id
optional extra information vector that is just passed through by CppAD,
and used by atomic_one derived class (not other derived classes).
This is an extra parameter to the virtual callbacks for atomic_one;
see the set_old member function.

\param ax
is the argument vector for this call,
<tt>ax.size()</tt> determines the number of arguments.

\param ay
is the result vector for this call,
<tt>ay.size()</tt> determines the number of results.
*/
template <class Base>
template <class ADVector>
void atomic_base<Base>::operator()(
    const ADVector&  ax     ,
          ADVector&  ay     ,
    size_t           id     )
{   size_t i, j;
    size_t n = ax.size();
    size_t m = ay.size();
# ifndef NDEBUG
    bool ok;
    std::string msg = "atomic_base: " + atomic_name() + ".eval: ";
    if( (n == 0) | (m == 0) )
    {   msg += "ax.size() or ay.size() is zero";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# endif
    size_t thread = thread_alloc::thread_num();
    allocate_work(thread);
    vector <Base>& tx  = work_[thread]->tx;
    vector <Base>& ty  = work_[thread]->ty;
    vector <bool>& vx  = work_[thread]->vx;
    vector <bool>& vy  = work_[thread]->vy;
    //
    if( vx.size() != n )
    {   vx.resize(n);
        tx.resize(n);
    }
    if( vy.size() != m )
    {   vy.resize(m);
        ty.resize(m);
    }
    //
    // Determine tape corresponding to variables in ax
    tape_id_t            tape_id  = 0;
    local::ADTape<Base>* tape     = nullptr;
    for(j = 0; j < n; j++)
    {   tx[j]  = ax[j].value_;
        vx[j]  = ! Constant( ax[j] );
        if( vx[j] )
        {
            if( tape_id == 0 )
            {   tape    = ax[j].tape_this();
                tape_id = ax[j].tape_id_;
                CPPAD_ASSERT_UNKNOWN( tape != nullptr );
            }
# ifndef NDEBUG
            if( tape_id != ax[j].tape_id_ )
            {   msg += atomic_name() +
                ": ax contains variables from different threads.";
                CPPAD_ASSERT_KNOWN(false, msg.c_str());
            }
# endif
        }
    }
    // Use zero order forward mode to compute values
    size_t p = 0, q = 0;
    set_old(id);
# ifdef NDEBUG
    forward(p, q, vx, vy, tx, ty);
# else
    ok = forward(p, q, vx, vy, tx, ty);
    if( ! ok )
    {   msg += atomic_name() + ": ok is false for "
            "zero order forward mode calculation.";
        CPPAD_ASSERT_KNOWN(false, msg.c_str());
    }
# endif
    bool record_operation = false;
    for(i = 0; i < m; i++)
    {
        // pass back values
        ay[i].value_ = ty[i];

        // initialize entire vector parameters (not in tape)
        ay[i].tape_id_ = 0;
        ay[i].taddr_   = 0;

        // we need to record this operation if
        // any of the elemnts of ay are variables,
        record_operation |= vy[i];
    }
# ifndef NDEBUG
    if( record_operation & (tape == nullptr) )
    {   msg +=
        "all elements of vx are false but vy contains a true element";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# endif
    // if tape is not null, ay is on the tape
    if( record_operation )
    {
        // Operator that marks beginning of this atomic operation
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::AFunOp) == 0 );
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::AFunOp) == 4 );
        CPPAD_ASSERT_KNOWN(
            size_t( std::numeric_limits<addr_t>::max() ) >=
            std::max( std::max( std::max(index_, id), n), m ),
            "atomic_base: cppad_tape_addr_type maximum not large enough"
        );
        tape->Rec_.PutArg(addr_t(index_), addr_t(id), addr_t(n), addr_t(m));
        tape->Rec_.PutOp(local::AFunOp);

        // Now put n operators, one for each element of argument vector
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::FunavOp) == 0 );
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::FunapOp) == 0 );
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::FunavOp) == 1 );
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::FunapOp) == 1 );
        for(j = 0; j < n; j++)
        {   if( Variable(ax[j]) )
            {   // information for an argument that is a variable
                tape->Rec_.PutArg(ax[j].taddr_);
                tape->Rec_.PutOp(local::FunavOp);
            }
            else
            {   // information for an argument that is parameter
                addr_t par = ax[j].taddr_;
                if( ! Dynamic( ax[j] ) )
                    par = tape->Rec_.put_con_par(ax[j].value_);
                tape->Rec_.PutArg(par);
                tape->Rec_.PutOp(local::FunapOp);
            }
        }

        // Now put m operators, one for each element of result vector
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::FunrpOp) == 1 );
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::FunrpOp) == 0 );
        CPPAD_ASSERT_UNKNOWN( local::NumArg(local::FunrvOp) == 0 );
        CPPAD_ASSERT_UNKNOWN( local::NumRes(local::FunrvOp) == 1 );
        for(i = 0; i < m; i++)
        {   if( vy[i] )
            {   ay[i].taddr_    = tape->Rec_.PutOp(local::FunrvOp);
                ay[i].tape_id_  = tape_id;
                ay[i].ad_type_  = variable_enum;
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( ! Dynamic( ay[i] ) );
                addr_t par = tape->Rec_.put_con_par(ay[i].value_);
                tape->Rec_.PutArg(par);
                tape->Rec_.PutOp(local::FunrpOp);
            }
        }

        // Put a duplicate AFunOp at end of AFunOp sequence
        CPPAD_ASSERT_KNOWN(
            size_t( std::numeric_limits<addr_t>::max() ) >=
            std::max( std::max( std::max(index_, id), n), m ),
            "atomic_base: cppad_tape_addr_type maximum not large enough"
        );
        tape->Rec_.PutArg(addr_t(index_), addr_t(id), addr_t(n), addr_t(m));
        tape->Rec_.PutOp(local::AFunOp);
    }
    return;
}


} // END_CPPAD_NAMESPACE
# endif
