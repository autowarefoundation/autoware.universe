# ifndef CPPAD_CORE_VALUE_HPP
# define CPPAD_CORE_VALUE_HPP
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
$begin Value$$
$spell
    const
$$



$section Convert From an AD Type to its Base Type$$

$head Syntax$$
$icode%b% = Value(%x%)%$$

$head See Also$$
$cref var2par$$


$head Purpose$$
Converts from an AD type to the corresponding
$cref/base type/glossary/Base Type/$$.

$head x$$
The argument $icode x$$ has prototype
$codei%
    const AD<%Base%> &%x%
%$$

$head b$$
The return value $icode b$$ has prototype
$codei%
    %Base% %b%
%$$

$head Operation Sequence$$
The result of this operation is not an
$cref/AD of Base/glossary/AD of Base/$$ object.
Thus it will not be recorded as part of an
AD of $icode Base$$
$cref/operation sequence/glossary/Operation/Sequence/$$.

$head Restriction$$
The argument $icode x$$ must not be a
$cref/variable/glossary/Variable/$$ or
$cref/dynamic/glossary/Parameter/Dynamic/$$ parameter
because its dependency information
would not be included in the $code Value$$ result $icode b$$.

$head Example$$
$children%
    example/general/value.cpp
%$$
The file
$cref value.cpp$$
contains an example and test of this operation.

$end
-------------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base>
CPPAD_INLINE_FRIEND_TEMPLATE_FUNCTION
Base Value(const AD<Base> &x)
{   Base result;
    //
    CPPAD_ASSERT_KNOWN(
        ! ( Variable(x) | Dynamic(x) ) ,
        "Value: argument is a variable or dynamic parameter"
    );
    //
    result = x.value_;
    return result;
}

}
//  END CppAD namespace


# endif
