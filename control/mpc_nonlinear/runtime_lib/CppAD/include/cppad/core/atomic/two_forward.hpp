# ifndef CPPAD_CORE_ATOMIC_TWO_FORWARD_HPP
# define CPPAD_CORE_ATOMIC_TWO_FORWARD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_two_forward$$
$spell
    sq
    mul.hpp
    hes
    afun
    vx
    vy
    ty
    Taylor
    const
    CppAD
    bool
    atx
    aty
    af
$$

$section Atomic Forward Mode$$


$head Syntax$$

$subhead Base$$
$icode%ok% = %afun%.forward(%p%, %q%, %vx%, %vy%, %tx%, %ty%)
%$$
This syntax is used by $icode%f%.Forward%$$ where $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
and $icode afun$$ is used in $icode f$$.

$subhead AD<Base>$$
$icode%ok% = %afun%.forward(%p%, %q%, %vx%, %vy%, %atx%, %aty%)
%$$
This syntax is used by $icode%af%.Forward%$$ where $icode af$$ has prototype
$codei%
    ADFun< AD<%Base%> , %Base% > %af%
%$$
and $icode afun$$ is used in $icode af$$ (see $cref base2ad$$).

$head Purpose$$
This virtual function is used by $cref atomic_two_afun$$
to evaluate function values.
It is also used buy
$cref/f.Forward/Forward/$$ (and $icode%af%.Forward%$$)
to compute function vales and derivatives.

$head Implementation$$
This virtual function must be defined by the
$cref/atomic_user/atomic_two_ctor/atomic_user/$$ class.
It can just return $icode%ok% == false%$$
(and not compute anything) for values
of $icode%q% > 0%$$ that are greater than those used by your
$cref/forward/Forward/$$ mode calculations.

$head p$$
The argument $icode p$$ has prototype
$codei%
    size_t %p%
%$$
It specifies the lowest order Taylor coefficient that we are evaluating.
During calls to $cref atomic_two_afun$$, $icode%p% == 0%$$.

$head q$$
The argument $icode q$$ has prototype
$codei%
    size_t %q%
%$$
It specifies the highest order Taylor coefficient that we are evaluating.
During calls to $cref atomic_two_afun$$, $icode%q% == 0%$$.

$head vx$$
The $code forward$$ argument $icode vx$$ has prototype
$codei%
    const CppAD::vector<bool>& %vx%
%$$
The case $icode%vx%.size() > 0%$$ only occurs while evaluating a call to
$cref atomic_two_afun$$.
In this case,
$icode%p% == %q% == 0%$$,
$icode%vx%.size() == %n%$$, and
for $latex j = 0 , \ldots , n-1$$,
$icode%vx%[%j%]%$$ is true if and only if
$icode%ax%[%j%]%$$ is a $cref/variable/glossary/Variable/$$
or $cref/dynamic parameter/glossary/Parameter/Dynamic/$$
in the corresponding call to
$codei%
    %afun%(%ax%, %ay%)
%$$
If $icode%vx%.size() == 0%$$,
then $icode%vy%.size() == 0%$$ and neither of these vectors
should be used.

$head vy$$
The $code forward$$ argument $icode vy$$ has prototype
$codei%
    CppAD::vector<bool>& %vy%
%$$
If $icode%vy%.size() == 0%$$, it should not be used.
Otherwise,
$icode%q% == 0%$$ and $icode%vy%.size() == %m%$$.
The input values of the elements of $icode vy$$
are not specified (must not matter).
Upon return, for $latex j = 0 , \ldots , m-1$$,
$icode%vy%[%i%]%$$ is true if and only if
$icode%ay%[%i%]%$$ is a variable
or dynamic parameter
(CppAD uses $icode vy$$ to reduce the necessary computations).

$head tx$$
The argument $icode tx$$ has prototype
$codei%
    const CppAD::vector<%Base%>& %tx%
%$$
and $icode%tx%.size() == (%q%+1)*%n%$$.
It is used by $icode%f%.Forward%$$ where $icode f$$ has type
$codei%ADFun<%Base%> %f%$$ and $icode afun$$ is used in $icode f$$.
For $latex j = 0 , \ldots , n-1$$ and $latex k = 0 , \ldots , q$$,
we use the Taylor coefficient notation
$latex \[
\begin{array}{rcl}
    x_j^k    & = & tx [ j * ( q + 1 ) + k ]
    \\
    X_j (t)  & = & x_j^0 + x_j^1 t^1 + \cdots + x_j^q t^q
\end{array}
\] $$
Note that superscripts represent an index for $latex x_j^k$$
and an exponent for $latex t^k$$.
Also note that the Taylor coefficients for $latex X(t)$$ correspond
to the derivatives of $latex X(t)$$ at $latex t = 0$$ in the following way:
$latex \[
    x_j^k = \frac{1}{ k ! } X_j^{(k)} (0)
\] $$

$head atx$$
The argument $icode atx$$ has prototype
$codei%
    const CppAD::vector< AD<%Base%> >& %atx%
%$$
Otherwise, $icode atx$$ specifications are the same as for $icode tx$$.

$head ty$$
The argument $icode ty$$ has prototype
$codei%
    CppAD::vector<%Base%>& %ty%
%$$
and $icode%tx%.size() == (%q%+1)*%m%$$.
It is set by $icode%f%.Forward%$$ where $icode f$$ has type
$codei%ADFun<%Base%> %f%$$ and $icode afun$$ is used in $icode f$$.
Upon return,
For $latex i = 0 , \ldots , m-1$$ and $latex k = 0 , \ldots , q$$,
$latex \[
\begin{array}{rcl}
    Y_i (t)  & = & f_i [ X(t) ]
    \\
    Y_i (t)  & = & y_i^0 + y_i^1 t^1 + \cdots + y_i^q t^q + o ( t^q )
    \\
    ty [ i * ( q + 1 ) + k ] & = & y_i^k
\end{array}
\] $$
where $latex o( t^q ) / t^q \rightarrow 0$$ as $latex t \rightarrow 0$$.
Note that superscripts represent an index for $latex y_j^k$$
and an exponent for $latex t^k$$.
Also note that the Taylor coefficients for $latex Y(t)$$ correspond
to the derivatives of $latex Y(t)$$ at $latex t = 0$$ in the following way:
$latex \[
    y_j^k = \frac{1}{ k ! } Y_j^{(k)} (0)
\] $$
If $latex p > 0$$,
for $latex i = 0 , \ldots , m-1$$ and $latex k = 0 , \ldots , p-1$$,
the input of $icode ty$$ satisfies
$latex \[
    ty [ i * ( q + 1 ) + k ] = y_i^k
\]$$
and hence the corresponding elements need not be recalculated.

$head aty$$
The argument $icode aty$$ has prototype
$codei%
    const CppAD::vector< AD<%Base%> >& %aty%
%$$
Otherwise, $icode aty$$ specifications are the same as for $icode ty$$.

$head ok$$
If the required results are calculated, $icode ok$$ should be true.
Otherwise, it should be false.

$head Discussion$$
For example, suppose that $icode%q% == 2%$$,
and you know how to compute the function $latex f(x)$$,
its first derivative $latex f^{(1)} (x)$$,
and it component wise Hessian $latex f_i^{(2)} (x)$$.
Then you can compute $icode ty$$ using the following formulas:
$latex \[
\begin{array}{rcl}
y_i^0 & = & Y(0)
        = f_i ( x^0 )
\\
y_i^1 & = & Y^{(1)} ( 0 )
        = f_i^{(1)} ( x^0 ) X^{(1)} ( 0 )
        = f_i^{(1)} ( x^0 ) x^1
\\
y_i^2
& = & \frac{1}{2 !} Y^{(2)} (0)
\\
& = & \frac{1}{2} X^{(1)} (0)^\R{T} f_i^{(2)} ( x^0 ) X^{(1)} ( 0 )
  +   \frac{1}{2} f_i^{(1)} ( x^0 ) X^{(2)} ( 0 )
\\
& = & \frac{1}{2} (x^1)^\R{T} f_i^{(2)} ( x^0 ) x^1
  +    f_i^{(1)} ( x^0 ) x^2
\end{array}
\] $$
For $latex i = 0 , \ldots , m-1$$, and $latex k = 0 , 1 , 2$$,
$latex \[
    ty [ i * (q + 1) + k ] = y_i^k
\] $$

$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/two_forward.hpp
Atomic forward mode
*/
/*!
Link from atomic_base to forward mode (for replacement by derived class)

\param p [in]
lowerest order for this forward mode calculation.

\param q [in]
highest order for this forward mode calculation.

\param vx [in]
if size not zero, which components of x are variables

\param vy [out]
if size not zero, which components of y are variables

\param tx [in]
Taylor coefficients corresponding to x for this calculation.

\param ty [out]
Taylor coefficient corresponding to y for this calculation

See the forward mode in user's documentation for atomic_two
*/
template <class Base>
bool atomic_base<Base>::forward(
    size_t                    p  ,
    size_t                    q  ,
    const vector<bool>&       vx ,
          vector<bool>&       vy ,
    const vector<Base>&       tx ,
          vector<Base>&       ty )
{   return false; }
/*!
Link from atomic_base to forward mode (for replacement by derived class)

\param p [in]
lowerest order for this forward mode calculation.

\param q [in]
highest order for this forward mode calculation.

\param vx [in]
if size not zero, which components of x are variables

\param vy [out]
if size not zero, which components of y are variables

\param atx [in]
Taylor coefficients corresponding to x for this calculation.

\param aty [out]
Taylor coefficient corresponding to y for this calculation

See the forward mode in user's documentation for atomic_two
*/
template <class Base>
bool atomic_base<Base>::forward(
    size_t                    p   ,
    size_t                    q   ,
    const vector<bool>&       vx  ,
          vector<bool>&       vy  ,
    const vector< AD<Base> >& atx ,
          vector< AD<Base> >& aty )
{   return false; }
/*!
Convert atomic_three interface to atomic_two interface

\param order_low [in]
lowerest order for this forward mode calculation.

\param order_up [in]
highest order for this forward mode calculation.

\param  type_x [in]
if size not zero, which components of x are variables

\param type_y [out]
if size not zero, which components of y are variables

\param taylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param taylor_y [out]
Taylor coefficient corresponding to y for this calculation

See the forward mode in user's documentation for atomic_three
*/
# define CPPAD_ATOMIC_BASE_MUSTDO 0
template <class Base>
bool atomic_base<Base>::forward(
    size_t                       order_low  ,
    size_t                       order_up   ,
    const vector<ad_type_enum>&  type_x     ,
    vector<ad_type_enum>&        type_y     ,
    const vector<Base>&          taylor_x   ,
    vector<Base>&                taylor_y   )
{   //
    // atomic_base::afun(ax, ay) calls bool version directly
    CPPAD_ASSERT_UNKNOWN( type_x.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( type_y.size() == 0 );
    //
# if CPPAD_ATOMIC_BASE_MUSTDO
    size_t thread = thread_alloc::thread_num();
    allocate_work(thread);
    vector <bool>& vx  = work_[thread]->vx;
    vector <bool>& vy  = work_[thread]->vy;
    vx.resize(type_x.size());
    vy.resize(type_y.size());
# else
    vector<bool> vx, vy;
# endif
    //
    bool ok = forward(order_low, order_up, vx, vy, taylor_x, taylor_y);
    //
    return ok;
}
# undef CPPAD_ATOMIC_BASE_MUSTDO
/*!
Convert atomic_three interface to atomic_two interface

\param order_low [in]
lowerest order for this forward mode calculation.

\param order_up [in]
highest order for this forward mode calculation.

\param  type_x [in]
if size not zero, which components of x are variables

\param type_y [out]
if size not zero, which components of y are variables

\param ataylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param ataylor_y [out]
Taylor coefficient corresponding to y for this calculation

See the forward mode in user's documentation for atomic_three
*/
template <class Base>
bool atomic_base<Base>::forward(
    size_t                       order_low  ,
    size_t                       order_up   ,
    const vector<ad_type_enum>&  type_x     ,
    vector<ad_type_enum>&        type_y     ,
    const vector< AD<Base> >&    ataylor_x  ,
    vector< AD<Base> >&          ataylor_y  )
{   //
    // atomic_base::afun(ax, ay) calls bool version directly
    CPPAD_ASSERT_UNKNOWN( type_x.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( type_y.size() == 0 );
    //
    vector<bool> vx, vy;
    bool ok = forward(order_low, order_up, vx, vy, ataylor_x, ataylor_y);
    //
    return ok;
}

} // END_CPPAD_NAMESPACE
# endif
