# ifndef CPPAD_LOCAL_IS_POD_HPP
# define CPPAD_LOCAL_IS_POD_HPP
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
$begin is_pod$$
$spell
    nullptr
    CppAD
    namespace
    bool
    inline
$$

$section The is_pod Template Function$$

$head Default Definition$$
The default template definition is that
$codei%
    is_pod<%Type%>()
%$$
is false for all types.

$head Fundamental Types$$
This file specializes $codei%is_pod<%Type%>%$$ to be true where $icode Type$$
is any of the c++11 fundamental types that hold data; i.e.,
$code void$$ and $code nullptr_t$$ are excluded.

$head Other Type$$
You can inform CppAD that a particular $icode Type$$ is plain old data by
defining
$codei%
    namespace CppAD { namespace local {
        template <> inline bool is_pod<%Type%>(void) { return true; }
    } }
%$$
$end
*/
namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
    //
    template <class T> inline bool is_pod(void) { return false; }
    // bool
    template <> inline bool is_pod<bool>(void)                   {return true;}
    // short
    template <> inline bool is_pod<short int>(void)              {return true;}
    template <> inline bool is_pod<unsigned short int>(void)     {return true;}
    // int
    template <> inline bool is_pod<int>(void)                    {return true;}
    template <> inline bool is_pod<unsigned int>(void)           {return true;}
    // long
    template <> inline bool is_pod<long int>(void)               {return true;}
    template <> inline bool is_pod<unsigned long int>(void)      {return true;}
    // long long
    template <> inline bool is_pod<long long int>(void)          {return true;}
    template <> inline bool is_pod<unsigned long long int>(void) {return true;}
    // Character types
    template <> inline bool is_pod<char>(void)                   {return true;}
    template <> inline bool is_pod<signed char>(void)            {return true;}
    template <> inline bool is_pod<unsigned char>(void)          {return true;}
    template <> inline bool is_pod<wchar_t>(void)                {return true;}
    template <> inline bool is_pod<char16_t>(void)               {return true;}
    template <> inline bool is_pod<char32_t>(void)               {return true;}
    // floating point types
    template <> inline bool is_pod<float>(void)                  {return true;}
    template <> inline bool is_pod<double>(void)                 {return true;}
    template <> inline bool is_pod<long double>(void)            {return true;}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
