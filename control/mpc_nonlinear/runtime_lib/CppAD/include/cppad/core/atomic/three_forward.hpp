# ifndef CPPAD_CORE_ATOMIC_THREE_FORWARD_HPP
# define CPPAD_CORE_ATOMIC_THREE_FORWARD_HPP
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
$begin atomic_three_forward$$
$spell
    taylor
    ataylor
    af
    afun
    enum
    CppAD
    aparameter
$$

$section Atomic Function Forward Mode$$

$head Base$$
This syntax and prototype are used by
$cref/afun(ax, ay)/atomic_three_afun/$$; see
$cref/Base/atomic_three_afun/Base/$$.
They are also used by
$icode%f%.Forward%$$ and $icode%f%.new_dynamic%$$
where $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
and $icode afun$$ is used during the recording of $icode f$$.

$subhead Syntax$$
$icode%ok% = %afun%.forward(
    %parameter_x%, %type_x%,
    %need_y%, %order_low%, %order_up%, %type_x%, %taylor_x%, %taylor_y%
)%$$

$subhead Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE_BASE%// END_PROTOTYPE_BASE%1
%$$

$head AD<Base>$$
This syntax and prototype are used by
$icode%af%.Forward%$$ and $icode%af%.new_dynamic%$$
where $icode af$$ has prototype
$codei%
    ADFun< AD<%Base%> , %Base% > %af%
%$$
and $icode afun$$ is used in $icode af$$ (see $cref base2ad$$).

$subhead Syntax$$
$icode%ok% = %afun%.forward(
    %parameter_x%, %type_x%,
    %need_y%, %order_low%, %order_up%, %type_x%, %ataylor_x%, %ataylor_y%
)%$$

$subhead Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE_AD_BASE%// END_PROTOTYPE_AD_BASE%1
%$$

$head Implementation$$
The $icode taylor_x$$, $icode taylor_y$$ version of this function
must be defined by the
$cref/atomic_user/atomic_three_ctor/atomic_user/$$ class.
It can just return $icode%ok% == false%$$
(and not compute anything) for values
of $icode%order_up%$$ that are greater than those used by your
$cref/forward/Forward/$$ mode calculations
(order zero must be implemented).

$head parameter_x$$
See $cref/parameter_x/atomic_three/parameter_x/$$.

$head aparameter_x$$
The specifications for $icode aparameter_x$$
is the same as for $cref/parameter_x/atomic_three/parameter_x/$$
(only the type of $icode ataylor_x$$ is different).

$head type_x$$
See $cref/type_x/atomic_three/type_x/$$.

$head need_y$$
One can ignore this argument and compute all the $icode taylor_y$$
Taylor coefficient.
Often, this is not necessary and $icode need_y$$ is used to specify this.
The value $cref/type_y/atomic_three_for_type/type_y/$$ is used
to determine which coefficients are necessary as follows:

$subhead Constant Parameters$$
If $icode%need_y% == size_t(constant_enum)%$$,
then only the taylor coefficients
for $latex Y_i (t)$$ where $icode%type_y%[%i%] == constant_enum%$$
are necessary.
This is the case during a $cref from_json$$ operation.

$subhead Dynamic Parameters$$
If $icode%need_y% == size_t(dynamic_enum)%$$,
then only the taylor coefficients
for $latex Y_i (t)$$ where $icode%type_y%[%i%] == dynamic_enum%$$
are necessary.
This is the case during an $cref new_dynamic$$ operation.

$subhead Variables$$
If $icode%need_y% == size_t(variable_enum)%$$,
If $codei%ad_type_enum(%need_y%)% == variable_enum%$$,
then only the taylor coefficients
for $latex Y_i (t)$$ where $icode%type_y%[%i%] == variable_enum%$$
are necessary.
This is the case during a $cref/f.Forward/Forward/$$ operation.
T

$subhead All$$
If $icode%need_y > size_t(variable_enum)%$$,
then the taylor coefficients for all $latex Y_i (t)$$ are necessary.
This is the case during an $icode%afun%(%ax%, %ay%)%$$ operation.


$head order_low$$
This argument
specifies the lowest order Taylor coefficient that we are computing.

$subhead p$$
We sometimes use the notation $icode%p% = %order_low%$$ below.

$head order_up$$
This argument
specifies the highest order Taylor coefficient that we are computing
($icode%order_low% <= %order_up%$$).

$subhead q$$
We sometimes use the notation $icode%q% = %order_up%$$ below.

$head taylor_x$$
The size of $icode taylor_x$$ is $codei%(%q%+1)*%n%$$.
For $latex j = 0 , \ldots , n-1$$ and $latex k = 0 , \ldots , q$$,
we use the Taylor coefficient notation
$latex \[
\begin{array}{rcl}
    x_j^k    & = & \R{taylor\_x} [ j * ( q + 1 ) + k ]
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

$subhead parameters$$
If the $th j$$ component of $icode x$$ corresponds to a parameter,
$codei%
    %type_x%[%j%] < CppAD::variable_enum
%$$
In this case,
the $th j$$ component of $icode parameter_x$$ is equal to $latex x_j^0$$;
i.e.,
$codei%
    %parameter_x%[%j%] == %taylor_x%[ %j% * ( %q% + 1 ) + 0 ]
%$$
Furthermore, for $icode%k% > 0%$$,
$codei%
    %taylor_x%[ %j% * ( %q% + 1 ) + %k% ] == 0
%$$

$head ataylor_x$$
The specifications for $icode ataylor_x$$ is the same as for $icode taylor_x$$
(only the type of $icode ataylor_x$$ is different).

$head taylor_y$$
The size of $icode taylor_y$$ is $codei%(%q%+1)*%m%$$.
Upon return,
For $latex i = 0 , \ldots , m-1$$ and $latex k = 0 , \ldots , q$$,
$latex \[
\begin{array}{rcl}
    Y_i (t)  & = & g_i [ X(t) ]
    \\
    Y_i (t)  & = & y_i^0 + y_i^1 t^1 + \cdots + y_i^q t^q + o ( t^q )
    \\
    \R{taylor\_y}  [ i * ( q + 1 ) + k ] & = & y_i^k
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
the input of $icode taylor_y$$ satisfies
$latex \[
    \R{taylor\_y}  [ i * ( q + 1 ) + k ] = y_i^k
\]$$
These values do not need to be recalculated
and can be used during the computation of the higher order coefficients.

$head ataylor_y$$
The specifications for $icode ataylor_y$$ is the same as for $icode taylor_y$$
(only the type of $icode ataylor_y$$ is different).

$head ok$$
If this calculation succeeded, $icode ok$$ is true.
Otherwise, it is false.

$head Discussion$$
For example, suppose that $icode%order_up% == 2%$$,
and you know how to compute the function $latex g(x)$$,
its first derivative $latex f^{(1)} (x)$$,
and it component wise Hessian $latex g_i^{(2)} (x)$$.
Then you can compute $icode taylor_x$$ using the following formulas:
$latex \[
\begin{array}{rcl}
y_i^0 & = & Y(0)
        = g_i ( x^0 )
\\
y_i^1 & = & Y^{(1)} ( 0 )
        = g_i^{(1)} ( x^0 ) X^{(1)} ( 0 )
        = g_i^{(1)} ( x^0 ) x^1
\\
y_i^2
& = & \frac{1}{2 !} Y^{(2)} (0)
\\
& = & \frac{1}{2} X^{(1)} (0)^\R{T} g_i^{(2)} ( x^0 ) X^{(1)} ( 0 )
  +   \frac{1}{2} g_i^{(1)} ( x^0 ) X^{(2)} ( 0 )
\\
& = & \frac{1}{2} (x^1)^\R{T} g_i^{(2)} ( x^0 ) x^1
  +    g_i^{(1)} ( x^0 ) x^2
\end{array}
\] $$
For $latex i = 0 , \ldots , m-1$$, and $latex k = 0 , 1 , 2$$,
$latex \[
    \R{taylor\_y} [ i * (q + 1) + k ] = y_i^k
\] $$

$children%
    example/atomic_three/forward.cpp%
    example/atomic_three/dynamic.cpp
%$$
$head Examples$$
The files
$cref atomic_three_forward.cpp$$ and $cref atomic_three_dynamic.cpp$$
contain examples and tests that uses this routine.

$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/three_forward.hpp
Third generation atomic forward mode.
*/
/*!
Link from atomic_three to forward mode

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
// BEGIN_PROTOTYPE_BASE
template <class Base>
bool atomic_three<Base>::forward(
    const vector<Base>&          parameter_x ,
    const vector<ad_type_enum>&  type_x      ,
    size_t                       need_y      ,
    size_t                       order_low   ,
    size_t                       order_up    ,
    const vector<Base>&          taylor_x    ,
    vector<Base>&                taylor_y    )
// END_PROTOTYPE_BASE
{   return false; }

/*!
Link from atomic_three to forward mode

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

See the forward mode in user's documentation for base_three
*/
// BEGIN_PROTOTYPE_AD_BASE
template <class Base>
bool atomic_three<Base>::forward(
    const vector< AD<Base> >&    aparameter_x ,
    const vector<ad_type_enum>&  type_x       ,
    size_t                       need_y       ,
    size_t                       order_low    ,
    size_t                       order_up     ,
    const vector< AD<Base> >&    ataylor_x    ,
    vector< AD<Base> >&          ataylor_y    )
// END_PROTOTYPE_AD_BASE
{   return false; }


} // END_CPPAD_NAMESPACE
# endif
