# ifndef CPPAD_CORE_ATOMIC_THREE_REVERSE_HPP
# define CPPAD_CORE_ATOMIC_THREE_REVERSE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin atomic_three_reverse$$
$spell
    sq
    mul.hpp
    afun
    ty
    px
    py
    Taylor
    const
    CppAD
    atx
    aty
    apx
    apy
    af
    aparameter
    enum
    azmul
    ataylor
    apartial
$$

$section Atomic Function Reverse Mode$$

$head Base$$
This syntax is used by $icode%f%.Reverse%$$ where $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
and $icode afun$$ is used in $icode f$$;
see $cref/Base/atomic_three_afun/Base/$$.

$subhead Syntax$$
$icode%ok% = %afun%.reverse(
    %parameter_x%, %type_x%,
    %order_up%, %taylor_x%, %taylor_y%, %partial_x%, %partial_y%
)
%$$

$subhead Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE_BASE%// END_PROTOTYPE_BASE%1
%$$

$head AD<Base>$$
This syntax is used by $icode%af%.Reverse%$$ where $icode af$$ has prototype
$codei%
    ADFun< AD<%Base%> , %Base% > %af%
%$$
and $icode afun$$ is used in $icode af$$ (see $cref base2ad$$).

$subhead Syntax$$
$icode%ok% = %afun%.reverse(
    %aparameter_x%, %type_x%,
    %order_up%, %ataylor_x%, %ataylor_y%, %apartial_x%, %apartial_y%
)
%$$

$subhead Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE_AD_BASE%// END_PROTOTYPE_AD_BASE%1
%$$

$head Implementation$$
This function must be defined if
$cref/afun/atomic_three_ctor/atomic_user/afun/$$ is
used to define an $cref ADFun$$ object $icode f$$,
and reverse mode derivatives are computed for $icode f$$.
It can return $icode%ok% == false%$$
(and not compute anything) for values
of $icode order_up$$ that are greater than those used by your
$cref/reverse/Reverse/$$ mode calculations.

$head parameter_x$$
See $cref/parameter_x/atomic_three/parameter_x/$$.

$head aparameter_x$$
The specifications for $icode aparameter_x$$
is the same as for $cref/parameter_x/atomic_three/parameter_x/$$
(only the type of $icode ataylor_x$$ is different).

$head type_x$$
See $cref/type_x/atomic_three/type_x/$$.

$head order_up$$
This argument specifies the highest order Taylor coefficient that
computing the derivative of.

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
For $latex i = 0 , \ldots , m-1$$ and $latex k = 0 , \ldots , q$$,
we use the Taylor coefficient notation
$latex \[
\begin{array}{rcl}
    Y_i (t)  & = & g_i [ X(t) ]
    \\
    Y_i (t)  & = & y_i^0 + y_i^1 t^1 + \cdots + y_i^q t^q + o ( t^q )
    \\
    y_i^k    & = & \R{taylor\_y} [ i * ( q + 1 ) + k ]
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

$head ataylor_y$$
The specifications for $icode ataylor_y$$ is the same as for $icode taylor_y$$
(only the type of $icode ataylor_y$$ is different).

$head F$$
We use the notation $latex \{ x_j^k \} \in \B{R}^{n \times (q+1)}$$ for
$latex \[
    \{ x_j^k \W{:} j = 0 , \ldots , n-1, k = 0 , \ldots , q \}
\]$$
We use the notation $latex \{ y_i^k \} \in \B{R}^{m \times (q+1)}$$ for
$latex \[
    \{ y_i^k \W{:} i = 0 , \ldots , m-1, k = 0 , \ldots , q \}
\]$$
We define the function
$latex F : \B{R}^{n \times (q+1)} \rightarrow \B{R}^{m \times (q+1)}$$ by
$latex \[
    y_i^k = F_i^k [ \{ x_j^k \} ]
\] $$
Note that
$latex \[
    F_i^0 ( \{ x_j^k \} ) = g_i ( X(0) )  = g_i ( x^0 )
\] $$
We also note that
$latex F_i^\ell ( \{ x_j^k \} )$$ is a function of
$latex x^0 , \ldots , x^\ell$$
and is determined by the derivatives of $latex g_i (x)$$
up to order $latex \ell$$.

$head G, H$$
We use $latex G : \B{R}^{m \times (q+1)} \rightarrow \B{R}$$
to denote an arbitrary scalar valued function of $latex \{ y_i^k \}$$.
We use $latex H : \B{R}^{n \times (q+1)} \rightarrow \B{R}$$
defined by
$latex \[
    H ( \{ x_j^k \} ) = G[ F( \{ x_j^k \} ) ]
\] $$

$head partial_y$$
The size of $icode partial_y$$ is $codei%(%q%+1)*%m%%$$.
For $latex i = 0 , \ldots , m-1$$, $latex k = 0 , \ldots , q$$,
$latex \[
    \R{partial\_y} [ i * (q + 1 ) + k ] = \partial G / \partial y_i^k
\] $$

$head apartial_y$$
The specifications for $icode apartial_y$$ is the same as for
$icode partial_y$$ (only the type of $icode apartial_y$$ is different).

$head partial_x$$
The size of $icode partial_x$$ is $codei%(%q%+1)*%n%%$$.
The input values of the elements of $icode partial_x$$
are not specified (must not matter).
Upon return,
for $latex j = 0 , \ldots , n-1$$ and $latex \ell = 0 , \ldots , q$$,
$latex \[
\begin{array}{rcl}
\R{partial\_x} [ j * (q + 1) + \ell ] & = & \partial H / \partial x_j^\ell
\\
& = &
( \partial G / \partial \{ y_i^k \} ) \cdot
    ( \partial \{ y_i^k \} / \partial x_j^\ell )
\\
& = &
\sum_{k=0}^q
\sum_{i=0}^{m-1}
( \partial G / \partial y_i^k ) ( \partial y_i^k / \partial x_j^\ell )
\\
& = &
\sum_{k=\ell}^q
\sum_{i=0}^{m-1}
\R{partial\_y}[ i * (q + 1 ) + k ] ( \partial F_i^k / \partial x_j^\ell )
\end{array}
\] $$
Note that we have used the fact that for $latex k < \ell$$,
$latex \partial F_i^k / \partial x_j^\ell = 0$$.

$subhead Short Circuit Operations$$
Note that if
$codei%IdenticalZero(%partial_y%[%i%*(%q%+1)+%k%])%$$ is true,
one does not need to compute $latex ( \partial F_i^k / \partial x_j^\ell )$$;
see $cref base_identical$$.
This can be used,
in a similar way to $cref/need_y/atomic_three_forward/need_y/$$,
to avoid unnecessary operations.

$subhead azmul$$
An $cref/optimized/optimize/$$ function will use zero
for values in $icode taylor_x$$ and $icode taylor_y$$ that are
not necessary in the current context.
If you divide by these values when computing
$latex ( \partial F_i^k / \partial x_j^\ell )$$ you could get an nan
if the corresponding value in $icode partial_y$$ is zero.
To be careful, if you do divide by
$icode taylor_x$$ or $icode taylor_y$$, use $cref azmul$$
for to avoid zero over zero calculations.


$head apartial_x$$
The specifications for $icode apartial_x$$ is the same as for
$icode partial_x$$ (only the type of $icode apartial_x$$ is different).

$head ok$$
If this calculation succeeded, $icode ok$$ is true.
Otherwise it is false.

$children%
    example/atomic_three/reverse.cpp
%$$
$head Examples$$
The file $cref atomic_three_reverse.cpp$$ contains an example and test
that uses this routine.

$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/three_reverse.hpp
Third Generation Atomic reverse mode.
*/
/*!
Link from reverse mode sweep to users routine.

\param parameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.

\param order_up [in]
highest order for this reverse mode calculation.

\param taylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param taylor_y [in]
Taylor coefficient corresponding to y for this calculation

\param partial_x [out]
Partials w.r.t. the x Taylor coefficients.

\param partial_y [in]
Partials w.r.t. the y Taylor coefficients.

See atomic_three_reverse mode use documentation
*/
// BEGIN_PROTOTYPE_BASE
template <class Base>
bool atomic_three<Base>::reverse(
    const vector<Base>&         parameter_x ,
    const vector<ad_type_enum>& type_x      ,
    size_t                      order_up    ,
    const vector<Base>&         taylor_x    ,
    const vector<Base>&         taylor_y    ,
    vector<Base>&               partial_x   ,
    const vector<Base>&         partial_y   )
// END_PROTOTYPE_BASE
{   return false; }

/*!
Link from reverse mode sweep to users routine.

\param aparameter_x [in]
contains the values, in afun(ax, ay), for arguments that are parameters.

\param type_x [in]
what is the type, in afun(ax, ay), for each component of x.


\param order_up [in]
highest order for this reverse mode calculation.

\param ataylor_x [in]
Taylor coefficients corresponding to x for this calculation.

\param ataylor_y [in]
Taylor coefficient corresponding to y for this calculation

\param apartial_x [out]
Partials w.r.t. the x Taylor coefficients.

\param apartial_y [in]
Partials w.r.t. the y Taylor coefficients.

See atomic_three_reverse mode use documentation
*/
// BEGIN_PROTOTYPE_AD_BASE
template <class Base>
bool atomic_three<Base>::reverse(
    const vector< AD<Base> >&       aparameter_x ,
    const vector<ad_type_enum>&     type_x       ,
    size_t                          order_up     ,
    const vector< AD<Base> >&       ataylor_x    ,
    const vector< AD<Base> >&       ataylor_y    ,
    vector< AD<Base> >&             apartial_x   ,
    const vector< AD<Base> >&       apartial_y   )
// END_PROTOTYPE_AD_BASE
{   return false; }

} // END_CPPAD_NAMESPACE
# endif
