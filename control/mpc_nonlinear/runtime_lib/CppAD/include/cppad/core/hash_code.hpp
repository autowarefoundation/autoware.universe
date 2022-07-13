# ifndef CPPAD_CORE_HASH_CODE_HPP
# define CPPAD_CORE_HASH_CODE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
\file core/hash_code.hpp
CppAD hashing utility.
*/
# include <cppad/local/hash_code.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
General purpose hash code for an arbitrary value.

\tparam Value
is the type of the argument being hash coded.
It should be a plain old data class; i.e.,
the values included in the equality operator in the object and
not pointed to by the object.

\param value
the value that we are generating a hash code for.
All of the fields in value should have been set before the hash code
is computed (otherwise undefined values are used).

\return
is a hash code that is between zero and CPPAD_HASH_TABLE_SIZE - 1.

\par Checked Assertions
\li std::numeric_limits<unsigned short>::max() >= CPPAD_HASH_TABLE_SIZE
\li sizeof(value) is even
\li sizeof(unsigned short)  == 2
*/
template <class Value>
unsigned short hash_code(const Value& value)
{   return local::local_hash_code(value); }

/*!
hash code for an AD<Base> object.

\tparam Base
is the base type for this AD value.

\param u
the AD value that we are generating a hash code for.

\return
is a hash code that is between zero and CPPAD_HASH_TABLE_SIZE - 1.
*/
template <class Base>
unsigned short hash_code(const AD<Base>& u)
{   size_t code = hash_code(u.value_);
    code       += size_t(u.taddr_);
    code       += size_t(u.ad_type_ == dynamic_enum);
    return (unsigned short)(code % CPPAD_HASH_TABLE_SIZE);
}

} // END_CPPAD_NAMESPACE


# endif
