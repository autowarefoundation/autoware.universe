# ifndef CPPAD_CORE_VAR2PAR_HPP
# define CPPAD_CORE_VAR2PAR_HPP
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
------------------------------------------------------------------------------

$begin Var2Par$$
$spell
    var
    const
$$


$section Convert an AD Variable or Dynamic Parameter to a Constant$$

$head Syntax$$
$icode%y% = Var2Par(%x%)%$$

$head See Also$$
$cref value$$

$head Purpose$$
Returns a
$cref/constant parameter/glossary/Parameter/Constant/$$ $icode y$$
with the same value as $icode x$$.

$head x$$
The argument $icode x$$ has prototype
$codei%
    const AD<%Base%> &x
%$$
The argument $icode x$$ may be a
variable, dynamic parameter, or constant parameter.

$head y$$
The result $icode y$$ has prototype
$codei%
    AD<%Base%> &y
%$$
and is a constant parameter.

$head Example$$
$children%
    example/general/var2par.cpp
%$$
The file
$cref var2par.cpp$$
contains an example and test of this operation.

$end
------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
AD<Base> Var2Par(const AD<Base> &x)
{   AD<Base> y(x.value_);
    return y;
}


template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
AD<Base> Var2Par(const VecAD_reference<Base> &x)
{   AD<Base> y(x.ADBase());
    y.id_ = 0;
}


} // END CppAD namespace

# endif
