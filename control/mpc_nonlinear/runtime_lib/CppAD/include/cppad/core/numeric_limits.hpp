# ifndef CPPAD_CORE_NUMERIC_LIMITS_HPP
# define CPPAD_CORE_NUMERIC_LIMITS_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
------------------------------------------------------------------------------
$begin numeric_limits$$
$spell
    std
    eps
    CppAD
    namespace
    const
    inf
    isnan
$$

$section Numeric Limits For an AD and Base Types$$

$head Syntax$$
$icode%eps% = numeric_limits<%Float%>::epsilon()
%$$
$icode%min% = numeric_limits<%Float%>::min()
%$$
$icode%max% = numeric_limits<%Float%>::max()
%$$
$icode%nan% = numeric_limits<%Float%>::quiet_NaN()
%$$
$icode%inf% = numeric_limits<%Float%>::infinity()
%$$
$codei%numeric_limits<%Float%>::digits10%$$

$head CppAD::numeric_limits$$
These functions and have the prototype
$codei%
    static %Float% CppAD::numeric_limits<%Float%>::%fun%(%void%)
%$$
where $icode fun$$ is
$code epsilon$$, $code min$$, $code max$$, $code quiet_NaN$$,
and $code infinity$$.
(Note that $code digits10$$ is member variable and not a function.)

$head std::numeric_limits$$
CppAD does not use a specialization of $code std::numeric_limits$$
because this would be to restrictive.
The C++ standard specifies that Non-fundamental standard
types, such as
$cref/std::complex<double>/base_complex.hpp/$$ shall not have specializations
of $code std::numeric_limits$$; see Section 18.2 of
ISO/IEC 14882:1998(E).
In addition, since C++11, a only literal types can have a specialization
of $code std::numeric_limits$$.

$head Float$$
These functions are defined for all $codei%AD<%Base%>%$$,
and for all corresponding $icode Base$$ types;
see $icode Base$$ type $cref base_limits$$.

$head epsilon$$
The result $icode eps$$ is equal to machine epsilon and has prototype
$codei%
    %Float% %eps%
%$$
The file $cref num_limits.cpp$$
tests the value $icode eps$$ by checking that the following are true
$codei%
    1 != 1 + %eps%
    1 == 1 + %eps% / 2
%$$
where all the values, and calculations, are done with the precision
corresponding to $icode Float$$.

$head min$$
The result $icode min$$ is equal to
the minimum positive normalized value and has prototype
$codei%
    %Float% %min%
%$$
The file $cref num_limits.cpp$$
tests the value $icode min$$ by checking that the following are true
$codei%
    abs( ((%min% / 100) * 100) / %min% - 1 ) > 3 * %eps%
    abs( ((%min% * 100) / 100) / %min% - 1 ) < 3 * %eps%
%$$
where all the values, and calculations, are done with the precision
corresponding to $icode Float$$.

$head max$$
The result $icode max$$ is equal to
the maximum finite value and has prototype
$codei%
    %Float% %max%
%$$
The file $cref num_limits.cpp$$
tests the value $icode max$$ by checking that the following are true
$codei%
    abs( ((%max% * 100) / 100) / %max% - 1 ) > 3 * %eps%
    abs( ((%max% / 100) * 100) / %max% - 1 ) < 3 * %eps%
%$$
where all the values, and calculations, are done with the precision
corresponding to $icode Float$$.

$head quiet_NaN$$
The result $icode nan$$ is not a number and has prototype
$codei%
    %Float% %nan%
%$$
The file $cref num_limits.cpp$$
tests the value $icode nan$$ by checking that the following is true
$codei%
    %nan% != %nan%
%$$

$head infinity$$
The result $icode inf$$ is equal to the
positive infinite value and has prototype
$codei%
    %Float% %inf%
%$$
The file $cref num_limits.cpp$$
tests the value $icode inf$$ by checking that the following are true
$codei%
    %inf% + 100 == %inf%
    isnan(%inf% - %inf%)
%$$

$head digits10$$
The member variable $code digits10$$ has prototype
$codei%
    static const int numeric_limits<%Float%>::digits10
%$$
It is the number of decimal digits that can be represented by a
$icode Float$$ value.  A number with this many decimal digits can be
converted to $icode Float$$ and back to a string,
without change due to rounding or overflow.


$head Example$$
$children%
    example/general/num_limits.cpp
%$$
The file
$cref num_limits.cpp$$
contains an example and test of these functions.

$end
------------------------------------------------------------------------------
*/
# include <iostream>

# include <cppad/configure.hpp>
# include <cppad/local/define.hpp>
# include <cppad/core/cppad_assert.hpp>
# include <cppad/local/declare_ad.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file numeric_limits.hpp
File that defines CppAD numeric_limits for AD types
*/

/// All tthese defaults correspond to errors
template <class Float>
class numeric_limits {
public:
    /// machine epsilon
    static Float epsilon(void)
    {   CPPAD_ASSERT_KNOWN(
        false,
        "numeric_limits<Float>::epsilon() is not specialized for this Float"
        );
        return Float(0);
    }
    /// minimum positive normalized value
    static Float min(void)
    {   CPPAD_ASSERT_KNOWN(
        false,
        "numeric_limits<Float>::min() is not specialized for this Float"
        );
        return Float(0);
    }
    /// maximum finite value
    static Float max(void)
    {   CPPAD_ASSERT_KNOWN(
        false,
        "numeric_limits<Float>::max() is not specialized for this Float"
        );
        return Float(0);
    }
    /// not a number
    static Float quiet_NaN(void)
    {   CPPAD_ASSERT_KNOWN(
        false,
        "numeric_limits<Float>::quiet_NaN() is not specialized for this Float"
        );
        return Float(0);
    }
    /// positive infinite value
    static Float infinity(void)
    {   CPPAD_ASSERT_KNOWN(
        false,
        "numeric_limits<Float>::infinity() is not specialized for this Float"
        );
        return Float(0);
    }
    /// number of decimal digits
    static const int digits10 = -1;
};

/// Partial specialization that defines limits for for all AD types
template <class Base>
class numeric_limits< AD<Base> > {
public:
    /// machine epsilon
    static AD<Base> epsilon(void)
    {   return AD<Base>( numeric_limits<Base>::epsilon() ); }
    /// minimum positive normalized value
    static AD<Base> min(void)
    {   return AD<Base>( numeric_limits<Base>::min() ); }
    /// maximum finite value
    static AD<Base> max(void)
    {   return AD<Base>( numeric_limits<Base>::max() ); }
    /// not a number
    static AD<Base> quiet_NaN(void)
    {   return AD<Base>( numeric_limits<Base>::quiet_NaN() ); }
    /// positive infinite value
    static AD<Base> infinity(void)
    {   return AD<Base>( numeric_limits<Base>::infinity() ); }
    /// number of decimal digits
    static const int digits10 = numeric_limits<Base>::digits10;
};

} // END_CPPAD_NAMESPACE
# endif
