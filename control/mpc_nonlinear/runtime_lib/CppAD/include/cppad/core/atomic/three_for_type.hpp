# ifndef CPPAD_CORE_ATOMIC_THREE_FOR_TYPE_HPP
# define CPPAD_CORE_ATOMIC_THREE_FOR_TYPE_HPP
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
$begin atomic_three_for_type$$
$spell
    afun
    enum
    cpp
$$

$section Atomic Function Forward Type Calculation$$

$head Syntax$$
$icode%ok% = %afun%.for_type(%parameter_x%, %type_x%, %type_y%)%$$

$subhead Prototype$$
$srcthisfile%0%// BEGIN_PROTOTYPE%// END_PROTOTYPE%1
%$$

$head Dependency Analysis$$
This calculation is sometimes referred to as a forward dependency analysis.

$head Usage$$
This syntax and prototype are used by
$codei%
    %afun%(%ax%, %ay%)
%$$
where $cref/afun/atomic_three_ctor/atomic_user/afun/$$
is a user defined atomic function.

$head Implementation$$
This virtual function must be defined by the
$cref/atomic_user/atomic_three_ctor/atomic_user/$$ class.

$head Base$$
See $cref/Base/atomic_three/Base/$$.

$head parameter_x$$
See $cref/parameter_x/atomic_three/parameter_x/$$.

$head type_x$$
See $cref/type_x/atomic_three/type_x/$$.

$head type_y$$
This vector has size equal to the number of results for this atomic function;
i.e. $icode%m%=%ay%.size()%$$.
The input values of the elements of $icode type_y$$
are not specified (must not matter).
Upon return, for $latex i = 0 , \ldots , m-1$$,
$icode%type_y%[%i%]%$$ is set to one of the following values:
$list number$$
It is $code constant_enum$$ if $icode%ay%[%i%]%$$ only depends on
the arguments that are constants.
$lnext
It is $code dynamic_enum$$ if $icode%ay%[%i%]%$$ depends on
a dynamic parameter and does not depend on any variables.
$lnext
It is $code variable_enum$$ if $icode%ay%[%i%]%$$ depends on
a variable.
$lend

$head ok$$
If this calculation succeeded, $icode ok$$ is true.
Otherwise, it is false.

$head Example$$
The following is an example of a atomic function $code for_type$$ definition:
$cref%get_started.cpp%atomic_three_get_started.cpp%for_type%$$.


$end
-----------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file atomic/three_for_type.hpp
Third generation atomic type computation.
*/
/*!
Link from atomic_three to type calculation

\param parameter_x [in]
is the value of the parameters in the corresponding function call
afun(ax, ay).

\param type_x [in]
specifies which components of x are
constants, dynamics, and variables

\param type_y [out]
specifies which components of y are
constants, dynamics, and variables
*/
// BEGIN_PROTOTYPE
template <class Base>
bool atomic_three<Base>::for_type(
    const vector<Base>&          parameter_x ,
    const vector<ad_type_enum>&  type_x      ,
    vector<ad_type_enum>&        type_y      )
// END_PROTOTYPE
{   return false; }

} // END_CPPAD_NAMESPACE

# endif
