# ifndef CPPAD_CORE_ATOMIC_THREE_REV_DEPEND_HPP
# define CPPAD_CORE_ATOMIC_THREE_REV_DEPEND_HPP
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
$begin atomic_three_rev_depend$$
$spell
    afun
    enum
    cpp
    taylor.hpp
$$

$section Atomic Function Reverse Dependency Calculation$$

$head Syntax$$
$icode%ok% = %afun%.rev_depend(
    %parameter_x%, %type_x%, %depend_x%, %depend_y%
)%$$

$subhead Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head Dependency Analysis$$
This calculation is sometimes referred to as a reverse dependency analysis.

$head Implementation$$
This function must be defined if
$cref/afun/atomic_three_ctor/atomic_user/afun/$$ is
used to define an $cref ADFun$$ object $icode f$$,
and $cref/f.optimize()/optimize/$$ is used.

$head Base$$
See $cref/Base/atomic_three_afun/Base/$$.

$head parameter_x$$
See $cref/parameter_x/atomic_three/parameter_x/$$.

$head type_x$$
See $cref/type_x/atomic_three/type_x/$$.

$head depend_x$$
This vector has size equal to the number of arguments for this atomic function;
i.e. $icode%n%=%ax%.size()%$$.
The input values of the elements of $icode depend_x$$
are not specified (must not matter).
Upon return, for $latex j = 0 , \ldots , n-1$$,
$icode%depend_x%[%j%]%$$ is true if the values of interest depend
on the value of $cref/ax[j]/atomic_three_afun/ax/$$ in the corresponding
$icode%afun%(%ax%, %ay%)%$$ call.
Note that parameters and variables,
that the values of interest do not depend on,
may get removed by $cref/optimization/optimize/$$.
The corresponding values in $cref/parameter_x/atomic_three/parameter_x/$$,
and $cref/taylor_x/atomic_three_forward/taylor_x/$$
(after optimization has removed them) are not specified.

$head depend_y$$
This vector has size equal to the number of results for this atomic function;
i.e. $icode%m%=%ay%.size()%$$.
For $latex i = 0 , \ldots , m-1$$,
$icode%depend_y%[%i%]%$$ is true if the values of interest depend
on the value of $cref/ay[i]/atomic_three_afun/ay/$$ in the corresponding
$icode%afun%(%ax%, %ay%)%$$ call.

$head ok$$
If this calculation succeeded, $icode ok$$ is true.
Otherwise, it is false.

$childtable%
    example/atomic_three/rev_depend.cpp
%$$
$head Example$$
The following is an example of a atomic function $code rev_depend$$ definition:
$cref atomic_three_rev_depend.cpp$$.


$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/three_rev_depend.hpp
Third generation atomic type computation.
*/
/*!
Link from atomic_three to reverse dependency calculation

\param parameter_x [in]
is the value of the parameters in the corresponding function call
afun(ax, ay).

\param type_x [in]
is the value for each of the components of x.

\param depend_x [out]
specifies which components of x affect values of interest.

\param depend_y [in]
specifies which components of y affect values of interest.
*/
// BEGIN_PROTOTYPE
template <class Base>
bool atomic_three<Base>::rev_depend(
    const vector<Base>&         parameter_x ,
    const vector<ad_type_enum>& type_x      ,
    vector<bool>&               depend_x    ,
    const vector<bool>&         depend_y    )
// END_PROTOTYPE
{   return false; }

} // END_CPPAD_NAMESPACE

# endif
