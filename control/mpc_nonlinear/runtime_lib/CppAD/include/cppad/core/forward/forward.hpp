# ifndef CPPAD_CORE_FORWARD_FORWARD_HPP
# define CPPAD_CORE_FORWARD_FORWARD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

// documened after Forward but included here so easy to see
# include <cppad/core/capacity_order.hpp>
# include <cppad/core/num_skip.hpp>
# include <cppad/core/check_for_nan.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*
--------------------------------------- ---------------------------------------
$begin devel_forward_order$$
$spell
    yq
    xq
    Taylor
    num_var
    PriOp
    dyn_ind
$$

$section Multiple orders, one direction, forward mode Taylor coefficients$$

$head Syntax$$
$icode%yq% = %f%.Forward(%q%, %xq%, %s% )
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_FORWARD_ORDER%// END_FORWARD_ORDER%1
%$$

$head Base$$
The type used during the forward mode computations; i.e., the corresponding
recording of operations used the type AD<Base>.

$head BaseVector$$
is a Simple Vector class with elements of type Base.

$head q$$
is the highest order for this forward mode computation; i.e.,
after this calculation there will be $icode%q%+1%$$
Taylor coefficients per variable.

$head xq$$
contains Taylor coefficients for the independent variables.
The size of $icode xq$$ must either be $icode n$$ or $icode (q+1)*n$$,
We define $icode p = q + 1 - xq.size()/n$$.
For $icode j = 0 , ... , n-1$$,
$icode k = 0, ... , q$$,
$icode xq[ (q+1)*j + k - p ]$$
is the k-th order coefficient for the j-th independent variable.

$head s$$
Is the stream where output corresponding to PriOp operations will written.

$head yq$$
contains Taylor coefficients for the dependent variables.
The size of the return value $icode yq$$
has size $icode m*(q+1-p)$$.
For $icode i = 0, ... , m-1$$,
$icode k = p, ..., q$$,
$icode yq[(q+1-p)*i + (k-p)]$$
is the k-th order coefficient for the i-th dependent variable.

$head taylor_$$
The Taylor coefficients up to order $icode p-1$$ are inputs
and the coefficients from order $icode p$$ through $icode q$$ are outputs.
Let $icode N = num_var_tape_$$, and
$icode C = cap_order_taylor_$$.
Note that for
$icode i = 1 , ..., N-1$$,
$icode k = 0 , ..., q$$,
$icode taylor_[ C*i + k ]$$
is the k-th order coefficient,
for the i-th variable on the tape.
(The first independent variable has index one on the tape
and there is no variable with index zero.)

$end
*/
// BEGIN_FORWARD_ORDER
template <class Base, class RecBase>
template <class BaseVector>
BaseVector ADFun<Base,RecBase>::Forward(
    size_t              q         ,
    const BaseVector&   xq        ,
          std::ostream& s         )
// END_FORWARD_ORDER
{
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);

    // temporary indices
    size_t i, j, k;

    // number of independent variables
    size_t n = ind_taddr_.size();

    // number of dependent variables
    size_t m = dep_taddr_.size();

    // check Vector is Simple Vector class with Base type elements
    CheckSimpleVector<Base, BaseVector>();

    // check size of xq
    CPPAD_ASSERT_KNOWN(
        size_t(xq.size()) == n || size_t(xq.size()) == n*(q+1),
        "Forward(q, xq): xq.size() is not equal n or n*(q+1)"
    );

    // p = lowest order we are computing
    size_t p = q + 1 - size_t(xq.size()) / n;
    CPPAD_ASSERT_UNKNOWN( p == 0 || p == q );

    // check one order case
    CPPAD_ASSERT_KNOWN(
        q <= num_order_taylor_ || p == 0,
        "Forward(q, xq): Number of Taylor coefficient orders stored in this"
        " ADFun\nis less than q and xq.size() != n*(q+1)."
    );

    // if p > 1, the previous number of directions must be one
    CPPAD_ASSERT_KNOWN(
        p <= 1 || num_direction_taylor_ == 1,
        "Forward(q, xq): computing order q >= 2"
        " and number of directions is not one."
        "\nMust use Forward(q, r, xq) for this case"
    );

    // does taylor_ need more orders or fewer directions
    if( (cap_order_taylor_ <= q) | (num_direction_taylor_ != 1) )
    {   if( p == 0 )
        {   // no need to copy old values during capacity_order
            num_order_taylor_ = 0;
        }
        else
            num_order_taylor_ = q;
        size_t c = std::max<size_t>(q + 1, cap_order_taylor_);
        size_t r = 1;
        capacity_order(c, r);
    }
    CPPAD_ASSERT_UNKNOWN( cap_order_taylor_ > q );
    CPPAD_ASSERT_UNKNOWN( num_direction_taylor_ == 1 );

    // short hand notation for order capacity
    size_t C = cap_order_taylor_;

    // The optimizer may skip a step that does not affect dependent variables.
    // Initilaizing zero order coefficients avoids following valgrind warning:
    // "Conditional jump or move depends on uninitialised value(s)".
    for(j = 0; j < num_var_tape_; j++)
    {   for(k = p; k <= q; k++)
            taylor_[C * j + k] = CppAD::numeric_limits<Base>::quiet_NaN();
    }

    // set Taylor coefficients for independent variables
    for(j = 0; j < n; j++)
    {   CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] < num_var_tape_  );

        // ind_taddr_[j] is operator taddr for j-th independent variable
        CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == local::InvOp );

        if( p == q )
            taylor_[ C * ind_taddr_[j] + q] = xq[j];
        else
        {   for(k = 0; k <= q; k++)
                taylor_[ C * ind_taddr_[j] + k] = xq[ (q+1)*j + k];
        }
    }

    // evaluate the derivatives
    CPPAD_ASSERT_UNKNOWN( cskip_op_.size() == play_.num_op_rec() );
    CPPAD_ASSERT_UNKNOWN( load_op2var_.size()  == play_.num_var_load_rec() );
    if( q == 0 )
    {
        local::sweep::forward0(&play_, s, true,
            n, num_var_tape_, C,
            taylor_.data(), cskip_op_.data(), load_op2var_,
            compare_change_count_,
            compare_change_number_,
            compare_change_op_index_,
            not_used_rec_base
        );
    }
    else
    {   local::sweep::forward1(&play_, s, true, p, q,
            n, num_var_tape_, C,
            taylor_.data(), cskip_op_.data(), load_op2var_,
            compare_change_count_,
            compare_change_number_,
            compare_change_op_index_,
            not_used_rec_base
        );
    }

    // return Taylor coefficients for dependent variables
    BaseVector yq;
    if( p == q )
    {   yq.resize(m);
        for(i = 0; i < m; i++)
        {   CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_  );
            yq[i] = taylor_[ C * dep_taddr_[i] + q];
        }
    }
    else
    {   yq.resize(m * (q+1) );
        for(i = 0; i < m; i++)
        {   for(k = 0; k <= q; k++)
                yq[ (q+1) * i + k] =
                    taylor_[ C * dep_taddr_[i] + k ];
        }
    }
# ifndef NDEBUG
    if( check_for_nan_ )
    {   bool ok = true;
        size_t index = m;
        if( p == 0 )
        {   for(i = 0; i < m; i++)
            {   // Visual Studio 2012, CppAD required in front of isnan ?
                if( CppAD::isnan( yq[ (q+1) * i + 0 ] ) )
                {   ok    = false;
                    if( index == m )
                        index = i;
                }
            }
        }
        if( ! ok )
        {   CPPAD_ASSERT_UNKNOWN( index < m );
            //
            CppAD::vector<Base> x0(n);
            for(j = 0; j < n; j++)
                x0[j] = taylor_[ C * ind_taddr_[j] + 0 ];
            std::string  file_name;
            put_check_for_nan(x0, file_name);
            std::stringstream ss;
            ss <<
            "yq = f.Forward(q, xq): a zero order Taylor coefficient is nan.\n"
            "Corresponding independent variables vector was written "
            "to binary a file.\n"
            "vector_size = " << n << "\n" <<
            "file_name = " << file_name << "\n" <<
            "index = " << index << "\n";
            // ss.str() returns a string object with a copy of the current
            // contents in the stream buffer.
            std::string msg_str       = ss.str();
            // msg_str.c_str() returns a pointer to the c-string
            // representation of the string object's value.
            const char* msg_char_star = msg_str.c_str();
            ErrorHandler::Call(
                true,
                __LINE__,
                __FILE__,
                "if( CppAD::isnan( yq[ (q+1) * index + 0 ] )",
                msg_char_star
            );
        }
        CPPAD_ASSERT_KNOWN(ok,
            "with the value nan."
        );
        if( 0 < q )
        {   for(i = 0; i < m; i++)
            {   for(k = p; k <= q; k++)
                {   // Studio 2012, CppAD required in front of isnan ?
                    ok &= ! CppAD::isnan( yq[ (q+1-p)*i + k-p ] );
                }
            }
        }
        CPPAD_ASSERT_KNOWN(ok,
        "yq = f.Forward(q, xq): has a non-zero order Taylor coefficient\n"
        "with the value nan (but zero order coefficients are not nan)."
        );
    }
# endif

    // now we have q + 1  taylor_ coefficient orders per variable
    num_order_taylor_ = q + 1;

    return yq;
}
/*
--------------------------------------- ---------------------------------------
$begin devel_forward_dir$$
$spell
    yq
    xq
    Taylor
    num_var
$$

$section One order, multiple directions, forward mode Taylor coefficients$$

$head Syntax$$
$icode%yq% = %f%.Forward(%q%, %r%, %xq%)
%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_FORWARD_DIR%// END_FORWARD_DIR%1
%$$

$head Base$$
The type used during the forward mode computations; i.e., the corresponding
recording of operations used the type AD<Base>.

$head BaseVector$$
is a Simple Vector class with elements of type Base.

$head q$$
is the order for this forward mode computation,
$icode q > 0$$.
There must be at least $icode q$$ Taylor coefficients
per variable before this call.
After this call there will be $icode q+1$$
Taylor coefficients per variable.

$head r$$
is the number of directions for this calculation.
If $icode q != 1$$, r must be the same as in the previous
call to Forward where q was equal to one.

$head xq$$
contains Taylor coefficients for the independent variables.
The size of xq must either be $icode r*n$$,
For $icode j = 0 , ... , n-1$$,
$icode ell = 0, ... , r-1$$,
$icode xq[ ( r*j + ell ]$$
is the q-th order coefficient for the j-th independent variable
and the ell-th direction.

$head yq$$
contains Taylor coefficients for the dependent variables.
The size of $icode y$$ is $icode r*m$$.
For $icode i = 0, ... , m-1$$,
$icode ell = 0, ... , r-1$$,
$icode yq[ r*i + ell ]$$
is the q-th order coefficient for the i-th dependent variable
and the ell-th direction.

$head taylor_$$
The Taylor coefficients up to order $icode q-1$$ are inputs
and the coefficients of order q are outputs.
Let $icode N = num_var_tape_$$, and
$icode C = cap_order_taylor_$$.
Note that for
$icode i = 1 , ..., N-1$$,
$icode taylor_[ (C-1)*r*i + i + 0 ]$$
is the zero order coefficient,
for the i-th variable, and all directions.
For $icode i = 1 , ..., N-1$$,
$icode k = 1 , ..., q$$,
$icode ell = 0 , ..., r-1$$,
$icode taylor_[ (C-1)*r*i + i + (k-1)*r + ell + 1 ]$$
is the k-th order coefficient,
for the i-th variable, and ell-th direction.
(The first independent variable has index one on the tape
and there is no variable with index zero.)

$end
*/
// BEGIN_FORWARD_DIR
template <class Base, class RecBase>
template <class BaseVector>
BaseVector ADFun<Base,RecBase>::Forward(
    size_t              q         ,
    size_t              r         ,
    const BaseVector&   xq        )
// END_FORWARD_DIR
{
    // used to identify the RecBase type in calls to sweeps
    RecBase not_used_rec_base(0.0);

    // temporary indices
    size_t i, j, ell;

    // number of independent variables
    size_t n = ind_taddr_.size();

    // number of dependent variables
    size_t m = dep_taddr_.size();

    // check Vector is Simple Vector class with Base type elements
    CheckSimpleVector<Base, BaseVector>();

    CPPAD_ASSERT_KNOWN( q > 0, "Forward(q, r, xq): q == 0" );
    CPPAD_ASSERT_KNOWN(
        size_t(xq.size()) == r * n,
        "Forward(q, r, xq): xq.size() is not equal r * n"
    );
    CPPAD_ASSERT_KNOWN(
        q <= num_order_taylor_ ,
        "Forward(q, r, xq): Number of Taylor coefficient orders stored in"
        " this ADFun is less than q"
    );
    CPPAD_ASSERT_KNOWN(
        q == 1 || num_direction_taylor_ == r ,
        "Forward(q, r, xq): q > 1 and number of Taylor directions r"
        " is not same as previous Forward(1, r, xq)"
    );

    // does taylor_ need more orders or new number of directions
    if( cap_order_taylor_ <= q || num_direction_taylor_ != r )
    {   if( num_direction_taylor_ != r )
            num_order_taylor_ = 1;

        size_t c = std::max<size_t>(q + 1, cap_order_taylor_);
        capacity_order(c, r);
    }
    CPPAD_ASSERT_UNKNOWN( cap_order_taylor_ > q );
    CPPAD_ASSERT_UNKNOWN( num_direction_taylor_ == r )

    // short hand notation for order capacity
    size_t c = cap_order_taylor_;

    // set Taylor coefficients for independent variables
    for(j = 0; j < n; j++)
    {   CPPAD_ASSERT_UNKNOWN( ind_taddr_[j] < num_var_tape_  );

        // ind_taddr_[j] is operator taddr for j-th independent variable
        CPPAD_ASSERT_UNKNOWN( play_.GetOp( ind_taddr_[j] ) == local::InvOp );

        for(ell = 0; ell < r; ell++)
        {   size_t index = ((c-1)*r + 1)*ind_taddr_[j] + (q-1)*r + ell + 1;
            taylor_[ index ] = xq[ r * j + ell ];
        }
    }

    // evaluate the derivatives
    CPPAD_ASSERT_UNKNOWN( cskip_op_.size() == play_.num_op_rec() );
    CPPAD_ASSERT_UNKNOWN( load_op2var_.size()  == play_.num_var_load_rec() );
    local::sweep::forward2(
        &play_,
        q,
        r,
        n,
        num_var_tape_,
        c,
        taylor_.data(),
        cskip_op_.data(),
        load_op2var_,
        not_used_rec_base
    );

    // return Taylor coefficients for dependent variables
    BaseVector yq;
    yq.resize(r * m);
    for(i = 0; i < m; i++)
    {   CPPAD_ASSERT_UNKNOWN( dep_taddr_[i] < num_var_tape_  );
        for(ell = 0; ell < r; ell++)
        {   size_t index = ((c-1)*r + 1)*dep_taddr_[i] + (q-1)*r + ell + 1;
            yq[ r * i + ell ] = taylor_[ index ];
        }
    }
# ifndef NDEBUG
    if( check_for_nan_ )
    {   bool ok = true;
        for(i = 0; i < m; i++)
        {   for(ell = 0; ell < r; ell++)
            {   // Studio 2012, CppAD required in front of isnan ?
                ok &= ! CppAD::isnan( yq[ r * i + ell ] );
            }
        }
        CPPAD_ASSERT_KNOWN(ok,
        "yq = f.Forward(q, r, xq): has a non-zero order Taylor coefficient\n"
        "with the value nan (but zero order coefficients are not nan)."
        );
    }
# endif

    // now we have q + 1  taylor_ coefficient orders per variable
    num_order_taylor_ = q + 1;

    return yq;
}


} // END_CPPAD_NAMESPACE
# endif
