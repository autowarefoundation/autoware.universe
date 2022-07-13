# ifndef CPPAD_CORE_AD_ASSIGN_HPP
# define CPPAD_CORE_AD_ASSIGN_HPP
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
------------------------------------------------------------------------------

$begin ad_assign$$
$spell
    Vec
    const
$$


$section AD Assignment Operator$$

$head Syntax$$
$icode%y% = %x%$$

$head Purpose$$
Assigns the value in $icode x$$ to the object $icode y$$.
In either case,

$head x$$
The argument $icode x$$ has prototype
$codei%
    const %Type% &%x%
%$$
where $icode Type$$ is
$codei%VecAD<%Base%>::reference%$$,
$codei%AD<%Base%>%$$,
$icode Base$$,
or any type that has an implicit constructor of the form
$icode%Base%(%x%)%$$.

$head y$$
The target $icode y$$ has prototype
$codei%
    AD<%Base%> %y%
%$$

$head Example$$
$children%
    example/general/ad_assign.cpp
%$$
The file $cref ad_assign.cpp$$ contain examples and tests of these operations.
It test returns true if it succeeds and false otherwise.

$end
------------------------------------------------------------------------------
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*!
\file ad_assign.hpp
AD<Base> constructors and and copy operations.
*/

/*!
\page AD_default_assign
Use default assignment operator
because they may be optimized better than the code below:
\code
template <class Base>
AD<Base>& AD<Base>::operator=(const AD<Base> &right)
{   value_    = right.value_;
    tape_id_  = right.tape_id_;
    taddr_    = right.taddr_;
    ad_type_  = right.ad_type_;

    return *this;
}
\endcode
*/

/*!
Assignment to Base type value.

\tparam Base
Base type for this AD object.

\param b
is the Base type value being assignment to this AD object.
The tape identifier will be an invalid tape identifier,
so this object is initially a parameter.
*/
template <class Base>
AD<Base>& AD<Base>::operator=(const Base &b)
{   value_   = b;
    tape_id_ = 0;
    //
    CPPAD_ASSERT_UNKNOWN( ! ( Variable(*this) | Dynamic(*this) ) );
    return *this;
}

/*!
Assignment to an ADVec<Base> element drops the vector information.

\tparam Base
Base type for this AD object.
*/
template <class Base>
AD<Base>& AD<Base>::operator=(const VecAD_reference<Base> &x)
{   *this = x.ADBase();
    CPPAD_ASSERT_UNKNOWN( ! Dynamic(*this) );
    return *this;
}

/*!
Assignment from any other type, converts to Base type, and then uses assignment
from Base type.

\tparam Base
Base type for this AD object.

\tparam T
is the the type that is being assigned to AD<Base>.
There must be an assignment for Base from Type.

\param t
is the object that is being assigned to an AD<Base> object.
*/
template <class Base>
template <class T>
AD<Base>& AD<Base>::operator=(const T &t)
{   *this = Base(t);
    CPPAD_ASSERT_UNKNOWN( ! ( Variable(*this) | Dynamic(*this) ) );
    return *this;
}


} // END_CPPAD_NAMESPACE
# endif
