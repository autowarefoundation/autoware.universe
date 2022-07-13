# ifndef CPPAD_LOCAL_SWEEP_CALL_ATOMIC_HPP
# define CPPAD_LOCAL_SWEEP_CALL_ATOMIC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/atomic_index.hpp>
# include <cppad/core/atomic/atomic_two.hpp>
# include <cppad/core/atomic/atomic_three.hpp>

// BEGIN_CPAPD_LOCAL_SWEEP_NAMESPACE
namespace CppAD { namespace local { namespace sweep {
/*!
\file call_atomic.hpp
Callbacks to atomic functions corresponding to atomic_index.
*/
// ----------------------------------------------------------------------------
/*!
Forward mode callback to atomic functions.

\tparam Base
Is the type corresponding to the Taylor coefficients.

\tparam RecBase
Is the type corresponding to this atomic function.

\param parameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param need_y
specifies which components of taylor_y are necessary.

\param order_low [in]
lowerest order for this forward mode calculation.

\param order_up [in]
highest order for this forward mode calculation.

\param atom_index [in]
is the index, in local::atomic_index, corresponding to this atomic function.

\param atom_old [in]
is the extra id information for this atomic function in the atomic_one case.

\param taylor_x [in]
Taylor coefficients corresponding to x.

\param taylor_y [out]
Taylor coefficient corresponding to y.
*/
template <class Base, class RecBase>
void call_atomic_forward(
    const vector<Base>&          parameter_x ,
    const vector<ad_type_enum>&  type_x      ,
    size_t                       need_y      ,
    size_t                       order_low   ,
    size_t                       order_up    ,
    size_t                       atom_index  ,
    size_t                       atom_old    ,
    const vector<Base>&          taylor_x    ,
    vector<Base>&                taylor_y    )
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            vector<ad_type_enum> empty;
            ok = afun->forward(
                order_low, order_up, empty, empty, taylor_x, taylor_y
            );
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->forward(
                parameter_x, type_x,
                need_y, order_low, order_up, taylor_x, taylor_y
            );
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(set_null, atom_index, type, &name, v_ptr);
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic forward returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
    if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        vector<ad_type_enum> empty;
        afun->set_old(atom_old);
        afun->forward(
            order_low, order_up, empty, empty, taylor_x, taylor_y
        );
    }
    else
    {   atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->forward(
            parameter_x, type_x,
            need_y, order_low, order_up, taylor_x, taylor_y
        );
    }
# endif
}
// ----------------------------------------------------------------------------
/*!
Reverse mode callback to atomic functions.

\tparam Base
Is the type corresponding to the Taylor coefficients.

\tparam RecBase
Is the type corresponding to this atomic function.

\param parameter_x [in]
value of the parameter arguments to the atomic function
(other arguments have the value nan).

\param type_x [in]
type for each component of x (not used by atomic_two interface).

\param order_up [in]
highest order for this reverse mode calculation.

\param atom_index [in]
is the index, in local::atomic_index, corresponding to this atomic function.

\param atom_old [in]
is the extra id information for this atomic function in the atomic_one case.

\param taylor_x [in]
Taylor coefficients corresponding to x.

\param taylor_y [in]
Taylor coefficient corresponding to y.

\param partial_x [out]
Partials w.r.t the x Taylor coefficients.

\param partial_y [in]
Partials w.r.t the y Taylor coefficients.
*/
template <class Base, class RecBase>
void call_atomic_reverse(
    const vector<Base>&          parameter_x ,
    const vector<ad_type_enum>&  type_x      ,
    size_t                       order_up    ,
    size_t                       atom_index  ,
    size_t                       atom_old    ,
    const vector<Base>&          taylor_x    ,
    const vector<Base>&          taylor_y    ,
    vector<Base>&                partial_x   ,
    const vector<Base>&          partial_y   )
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            ok = afun->reverse(
                order_up, taylor_x, taylor_y, partial_x, partial_y
            );
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->reverse(
                parameter_x, type_x,
                order_up, taylor_x, taylor_y, partial_x, partial_y
            );
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(set_null, atom_index, type, &name, v_ptr);
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic reverse returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
    if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        afun->set_old(atom_old);
        afun->reverse(
            order_up, taylor_x, taylor_y, partial_x, partial_y
        );
    }
    else
    {   atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->reverse(
            parameter_x, type_x,
            order_up, taylor_x, taylor_y, partial_x, partial_y
        );
    }
# endif
}
// ----------------------------------------------------------------------------
/*!
Forward Jacobian sparsity callback to atomic functions.

\tparam Base
is the type corresponding to parameter_x
and to this atomic function.

\tparam InternalSparsity
is the internal type used to represent sparsity; i.e.,
sparse::pack_setvec or sparse::list_setvec.

\param atom_index [in]
is the index, in local::atomic_index, corresponding to this atomic function.

\param atom_old [in]
is the extra id information for this atomic function in the atomic_one case.

\param dependency [in]
is this a dependency or sparsity calculation.

\param parameter_x [in]
value of the parameter arguments to the atomic function
(other arguments have the value nan).

\param type_x [in]
type for each component of x (not used by atomic_two interface).

\param x_index [in]
is a mapping from the index of an atomic function argument
to the corresponding variable on the tape.

\param y_index [in]
is a mapping from the index of an atomic function result
to the corresponding variable on the tape.

\param var_sparsity [in/out]
On input, for j = 0, ... , n-1, the sparsity pattern with index x_index[j],
is the sparsity for the j-th argument to this atomic function.
On output, for i = 0, ... , m-1, the sparsity pattern with index y_index[i],
is the sparsity for the j-th result for this atomic function.
*/
template <class Base, class RecBase, class InternalSparsity>
void call_atomic_for_jac_sparsity(
    size_t                       atom_index    ,
    size_t                       atom_old      ,
    bool                         dependency    ,
    const vector<Base>&          parameter_x   ,
    const vector<ad_type_enum>&  type_x        ,
    const pod_vector<size_t>&    x_index       ,
    const pod_vector<size_t>&    y_index       ,
    InternalSparsity&            var_sparsity  )
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if ( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            ok = afun->for_sparse_jac(
                parameter_x, x_index, y_index, var_sparsity
            );
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->for_jac_sparsity(
            dependency, parameter_x, type_x, x_index, y_index, var_sparsity
            );
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(
            set_null, atom_index, type, &name, v_ptr
        );
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic jac_sparsity returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
   if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        afun->set_old(atom_old);
        afun->for_sparse_jac(
            parameter_x, x_index, y_index, var_sparsity
        );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( type == 3 );
        atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->for_jac_sparsity(
            dependency, parameter_x, type_x, x_index, y_index, var_sparsity
        );
    }
# endif
}
// ----------------------------------------------------------------------------
/*!
Reverse Jacobian sparsity callback to atomic functions.

\tparam Base
is the type corresponding to parameter_x
and to this atomic function.

\tparam InternalSparsity
is the internal type used to represent sparsity; i.e.,
sparse::pack_setvec or sparse::list_setvec.

\param atom_index [in]
is the index, in local::atomic_index, corresponding to this atomic function.

\param atom_old [in]
is the extra id information for this atomic function in the atomic_one case.

\param dependency [in]
is this a dependency or sparsity calculation.

\param parameter_x [in]
value of the parameter arguments to the atomic function
(other arguments have the value nan).

\param type_x [in]
type for each component of x (not used by atomic_two interface).

\param x_index [in]
is a mapping from the index of an atomic function argument
to the corresponding variable on the tape.

\param y_index [in]
is a mapping from the index of an atomic function result
to the corresponding variable on the tape.

\param var_sparsity [in/out]
On input, for i = 0, ... , m-1, the sparsity pattern with index y_index[i],
is the sparsity for the i-th argument to this atomic function.
On output, for j = 0, ... , n-1, the sparsity pattern with index x_index[j],
the sparsity has been updated to remove y as a function of x.
*/
template <class Base, class RecBase, class InternalSparsity>
void call_atomic_rev_jac_sparsity(
    size_t                       atom_index    ,
    size_t                       atom_old      ,
    bool                         dependency    ,
    const vector<Base>&          parameter_x   ,
    const vector<ad_type_enum>&  type_x        ,
    const pod_vector<size_t>&    x_index       ,
    const pod_vector<size_t>&    y_index       ,
    InternalSparsity&            var_sparsity  )
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            ok = afun->rev_sparse_jac(
                parameter_x, x_index, y_index, var_sparsity
            );
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->rev_jac_sparsity(
            dependency, parameter_x, type_x, x_index, y_index, var_sparsity
            );
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(
            set_null, atom_index, type, &name, v_ptr
        );
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic jac_sparsity returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
    if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        afun->set_old(atom_old);
        afun->rev_sparse_jac(
            parameter_x, x_index, y_index, var_sparsity
        );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( type == 3 );
        atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->rev_jac_sparsity(
            dependency, parameter_x, type_x, x_index, y_index, var_sparsity
        );
    }
# endif
}
// ----------------------------------------------------------------------------
/*
$begin call_atomic_for_hes_sparsiy$$
$spell
    hes
    np
    numvar
    jac
    Jacobian
    afun
    setvec
$$

$section Forward Hessian Sparsity Callback to Atomic Functions.$$

$head Syntax$$
$codei%call_atomic_for_hes_sparsity(
    %atom_index%, %atom_old%, %parameter_x%, %type_x%, %x_index%, %y_index%,
    %np1%, %numvar%, %rev_jac_sparsity%, %for_sparsity%
)%$$

$head Prototype$$
$srcthisfile%
0%// BEGIN_call_atomic_for_hes_sparsity%// END_call_atomic_for_hes_sparsity%1
%$$

$head C++ Source$$
The C++ source code corresponding to this operation is a
$cref/atomic function call/atomic_three/Syntax/Use Atomic Function/$$
$codei%
    %afun%(%ax%, %ay%)
%$$
We refer to the corresponding function using $latex y = f(x)$$.

$head Base$$
is the type corresponding to $icode parameter_x$$
and to this atomic function.

$head InternalSparsity$$
is the internal type used to represent sparsity; i.e.,
$code sparse::pack_setvec$$ or $code sparse::list_setvec$$.

$head atom_index$$
is the index, in local::atomic_index, corresponding to this atomic function.

$head atom_old$$
is the extra id information for this atomic function in the atomic_one case.

$head parameter_x$$
value of the parameter arguments to the atomic function
(other arguments have the value nan).

$head type_x$$
type for each component of x (not used by atomic_two interface).

$head x_index$$
is a mapping from the index of an atomic function argument
to the corresponding variable on the tape.
We use $icode m_x$$ to denote the maximum value w.r.t $icode i$$ of
$icode%x_index%[%i%]%$$.

$head y_index$$
is a mapping from the index of an atomic function result
to the corresponding variable on the tape.
It should hold that $icode%m_i% < y_index%[%i%]%$$ for all $icode i$$.

$head np1$$
This is the number of independent variables plus one;
i.e. size of $icode x$$ plus one.

$head numvar$$
This is the total number of variables in the tape.

$head rev_jac_sparsity$$
For i = 0, ... , m-1, the sparsity pattern with index y_index[i],
is the reverse Jacobian sparsity for the i-th result to this atomic function.
This shows which components of the result affect the function we are
computing the Hessian of.

$head for_sparsity$$
We have the conditions $icode%np1% = %for_sparsity%.end()%$$
and $icode%for_sparsity%.n_set() = %np1% + %numvar%$$.

$subhead Input Jacobian Sparsity$$
For $icode%i%= 0, ..., %m_x%$$,
the $icode%np1%+%i%$$ row of $icode for_sparsity$$ is the Jacobian sparsity
for the $th i$$ variable. These values do not change.
Note that $icode%i%=0%$$ corresponds to a parameter and
the corresponding Jacobian sparsity is empty.

$subhead Input Hessian Sparsity$$
For $icode%i%=1, ..., %n%$$,
the $th i$$ row of $icode for_sparsity$$ is the Hessian sparsity
before including the function $latex y = f(x)$$.

$subhead Output Jacobian Sparsity$$
For $icode%i%=0, ..., %y_index%.size()%$$,
row $icode%np1%+%y_index%[%i%]%$$
of $icode for_sparsity$$ is the Jacobian sparsity
for the variable with index $icode%y_index%[%i%]%$$.

$subhead Output Hessian Sparsity$$
For $icode%i%=1, ..., %n%$$,
the $th i$$ row of $icode for_sparsity$$ is the Hessian sparsity
after including the function $latex y = f(x)$$.

$end
*/
// BEGIN_call_atomic_for_hes_sparsity
template <class Base, class RecBase, class InternalSparsity>
void call_atomic_for_hes_sparsity(
    size_t                       atom_index        ,
    size_t                       atom_old          ,
    const vector<Base>&          parameter_x       ,
    const vector<ad_type_enum>&  type_x            ,
    const pod_vector<size_t>&    x_index           ,
    const pod_vector<size_t>&    y_index           ,
    size_t                       np1               ,
    size_t                       numvar            ,
    const InternalSparsity&      rev_jac_sparsity  ,
    InternalSparsity&            for_sparsity      )
// END_call_atomic_for_hes_sparsity
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    CPPAD_ASSERT_UNKNOWN( for_sparsity.end() == np1 );
    CPPAD_ASSERT_UNKNOWN( for_sparsity.n_set() == np1 + numvar );

    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            ok = afun->for_sparse_hes(
                parameter_x,
                x_index,
                y_index,
                np1,
                numvar,
                rev_jac_sparsity,
                for_sparsity
            );
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->for_hes_sparsity(
                parameter_x,
                type_x,
                x_index,
                y_index,
                np1,
                numvar,
                rev_jac_sparsity,
                for_sparsity
            );
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(
            set_null, atom_index, type, &name, v_ptr
        );
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic hes_sparsity returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
    if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        afun->set_old(atom_old);
        afun->for_sparse_hes(
            parameter_x,
            x_index,
            y_index,
            np1,
            numvar,
            rev_jac_sparsity,
            for_sparsity
        );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( type == 3 );
        atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->for_hes_sparsity(
            parameter_x,
            type_x,
            x_index,
            y_index,
            np1,
            numvar,
            rev_jac_sparsity,
            for_sparsity
        );
    }
# endif
}
// ----------------------------------------------------------------------------
/*!
Reverse Hessian sparsity callback to atomic functions.

\tparam Base
is the type corresponding to parameter_x
and to this atomic function.

\tparam InternalSparsity
is the internal type used to represent sparsity; i.e.,
sparse::pack_setvec or sparse::list_setvec.

\param atom_index [in]
is the index, in local::atomic_index, corresponding to this atomic function.

\param atom_old [in]
is the extra id information for this atomic function in the atomic_one case.

\param parameter_x [in]
value of the parameter arguments to the atomic function
(other arguments have the value nan).

\param type_x [in]
type for each component of x (not used by atomic_two interface).

\param x_index [in]
is a mapping from the index of an atomic function argument
to the corresponding variable on the tape.

\param y_index [in]
is a mapping from the index of an atomic function result
to the corresponding variable on the tape.

\param for_jac_sparsity
For j = 0, ... , n-1, the sparsity pattern with index x_index[j],
is the forward Jacobian sparsity for the j-th argument to this atomic function.

\param rev_jac_flag
On input, for i = 0, ... , m-1, rev_jac_flag[ y_index[i] ] is true
if the fuction (we are computing the sparsity for)
depends on the variable y_index[i].
Upon return, for j = 0, ..., n-1, rev_jac_flag[ x_index[j] ] has been set to
true any of the y_index variables are flagged depnend on x_index[j].
Otherwise, rev_jac_flag[ x_index[j] ] is not modified.

\param rev_hes_sparsity
This is the sparsity pattern for the Hessian.
On input, for i = 0, ... , m-1, row y_index[i] is the reverse Hessian sparsity
with one of the partials with respect to to y_index[i].
Upon return, for j = 0, ..., n-1, the row x_index[j] has been
modified to include components that have a non-zero hessian through
the atomic fucntion with one of the partials w.r.t. x_index[j].
*/
template <class Base, class RecBase, class InternalSparsity>
void call_atomic_rev_hes_sparsity(
    size_t                       atom_index        ,
    size_t                       atom_old          ,
    const vector<Base>&          parameter_x       ,
    const vector<ad_type_enum>&  type_x            ,
    const pod_vector<size_t>&    x_index           ,
    const pod_vector<size_t>&    y_index           ,
    const InternalSparsity&      for_jac_sparsity  ,
    bool*                        rev_jac_flag      ,
    InternalSparsity&            rev_hes_sparsity  )
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            ok = afun->rev_sparse_hes(
                parameter_x,
                x_index,
                y_index,
                for_jac_sparsity,
                rev_jac_flag,
                rev_hes_sparsity
            );
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->rev_hes_sparsity(
                parameter_x,
                type_x,
                x_index,
                y_index,
                for_jac_sparsity,
                rev_jac_flag,
                rev_hes_sparsity
            );
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(
            set_null, atom_index, type, &name, v_ptr
        );
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic hes_sparsity returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
    if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        afun->set_old(atom_old);
        afun->rev_sparse_hes(
            parameter_x,
            x_index,
            y_index,
            for_jac_sparsity,
            rev_jac_flag,
            rev_hes_sparsity
        );
    }
    else
    {   CPPAD_ASSERT_UNKNOWN( type == 3 );
        atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->rev_hes_sparsity(
            parameter_x,
            type_x,
            x_index,
            y_index,
            for_jac_sparsity,
            rev_jac_flag,
            rev_hes_sparsity
        );
    }
# endif
}
// ----------------------------------------------------------------------------
/*!
Reverse dependency callback to atomic functions.

\param atom_index [in]
is the index, in local::atomic_index, corresponding to this atomic function.

\param atom_old [in]
is the extra id information for this atomic function in the atomic_one case.

\param parameter_x [in]
is the value of the parameters in the corresponding function call
afun(ax, ay).

\param type_x [in]
is the type for each x component in the corresponding function call
afun(ax, ay).

\param depend_x [out]
specifies which components of x affect values we are interested in.

\param depend_y [in]
specifies which components of y affect values we are interested in.
*/
template <class Base, class RecBase>
void call_atomic_rev_depend(
    size_t                      atom_index   ,
    size_t                      atom_old     ,
    const vector<Base>&         parameter_x  ,
    const vector<ad_type_enum>& type_x       ,
    vector<bool>&               depend_x     ,
    const vector<bool>&         depend_y     )
{   CPPAD_ASSERT_UNKNOWN( 0 < atom_index );
    bool         set_null = false;
    size_t       type     = 0;          // set to avoid warning
    std::string* name_ptr = nullptr;
    void*        v_ptr    = nullptr; // set to avoid warning
    local::atomic_index<RecBase>(set_null, atom_index, type, name_ptr, v_ptr);
# ifndef NDEBUG
    bool ok = v_ptr != nullptr;
    if( ok )
    {
        if( type == 2 )
        {   atomic_base<RecBase>* afun =
                reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
            afun->set_old(atom_old);
            vector<ad_type_enum> empty;
            ok = afun->rev_depend(parameter_x, type_x, depend_x, depend_y);
        }
        else
        {   CPPAD_ASSERT_UNKNOWN( type == 3 );
            atomic_three<RecBase>* afun =
                reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
            ok = afun->rev_depend(parameter_x, type_x, depend_x, depend_y);
        }
    }
    if( ! ok )
    {   // now take the extra time to copy the name
        std::string name;
        local::atomic_index<RecBase>(set_null, atom_index, type, &name, v_ptr);
        std::string msg = name;
        if( v_ptr == nullptr )
            msg += ": this atomic_three function has been deleted";
        else
            msg += ": atomic rev_depend returned false";
        CPPAD_ASSERT_KNOWN(false, msg.c_str() );
    }
# else
    if( type == 2 )
    {   atomic_base<RecBase>* afun =
            reinterpret_cast< atomic_base<RecBase>* >(v_ptr);
        vector<ad_type_enum> empty;
        afun->set_old(atom_old);
        afun->rev_depend(parameter_x, type_x, depend_x, depend_y);
    }
    else
    {   atomic_three<RecBase>* afun =
            reinterpret_cast< atomic_three<RecBase>* >(v_ptr);
        afun->rev_depend(parameter_x, type_x, depend_x, depend_y);
    }
# endif
}


} } } // END_CPAPD_LOCAL_SWEEP_NAMESPACE
# endif
