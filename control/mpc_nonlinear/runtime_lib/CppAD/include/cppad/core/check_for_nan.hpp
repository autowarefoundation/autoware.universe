# ifndef CPPAD_CORE_CHECK_FOR_NAN_HPP
# define CPPAD_CORE_CHECK_FOR_NAN_HPP
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
$begin check_for_nan$$
$spell
    std
    vec
    Cpp
    const
    bool
    newline
$$
$section Check an ADFun Object For Nan Results$$

$head Syntax$$
$icode%f%.check_for_nan(%b%)
%$$
$icode%b% = %f%.check_for_nan()
%$$
$codei%get_check_for_nan(%vec%, %file%)
%$$

$head Debugging$$
If $code NDEBUG$$ is not defined, and
the result of a $cref/forward/forward_order/$$ or $cref/reverse/reverse_any/$$
calculation contains a  $cref nan$$,
CppAD can halt with an error message.

$head f$$
For the syntax where $icode b$$ is an argument,
$icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
(see $codei%ADFun<%Base%>%$$ $cref/constructor/FunConstruct/$$).
For the syntax where $icode b$$ is the result,
$icode f$$ has prototype
$codei%
    const ADFun<%Base%> %f%
%$$

$head b$$
This argument or result has prototype
$codei%
    bool %b%
%$$
If $icode b$$ is true (false),
future calls to $icode%f%.Forward%$$ will (will not) check for $code nan$$.

$head Default$$
The value for this setting after construction of $icode f$$ is true.
The value of this setting is not affected by calling
$cref Dependent$$ for this function object.

$head Error Message$$
If this error is detected during zero order forward mode,
the values of the independent variables that resulted in the $code nan$$
are written to a temporary binary file.
This is so that you can run the original source code with those values
to see what is causing the $code nan$$.

$subhead vector_size$$
The error message with contain the text
$codei%vector_size = %vector_size%$$ followed the newline character
$code '\n'$$.
The value of $icode vector_size$$ is the number of elements
in the independent vector.

$subhead file_name$$
The error message with contain the text
$codei%file_name = %file_name%$$ followed the newline character
$code '\n'$$.
The value of $icode file_name$$ is the name of the temporary file
that contains the dependent variable values.

$subhead index$$
The error message will contain the text
$codei%index = %index%$$ followed by the newline character $code '\n'$$.
The value of $icode index$$ is the lowest dependent variable index
that has the value $code nan$$.

$head get_check_for_nan$$
This routine can be used to get the independent variable
values that result in a $code nan$$.

$subhead vec$$
This argument has prototype
$codei%
    CppAD::vector<%Base%>& %vec%
%$$
It size must be equal to the corresponding value of
$cref/vector_size/check_for_nan/Error Message/vector_size/$$
in the corresponding error message.
The input value of its elements does not matter.
Upon return, it will contain the values for the independent variables,
in the corresponding call to $cref Independent$$,
that resulted in the $code nan$$.
(Note that the call to $code Independent$$ uses an vector with elements
of type $codei%AD<%Base%>%$$ and $icode vec$$ has elements of type
$icode Base$$.)

$subhead file$$
This argument has prototype
$codei%
    const std::string& %file%
%$$
It must be the value of
$cref/file_name/check_for_nan/Error Message/file_name/$$
in the corresponding error message.

$head Example$$
$children%
    example/general/check_for_nan.cpp
%$$
The file
$cref check_for_nan.cpp$$
contains an example and test of these operations.

$end
*/

# include <cppad/utility/vector.hpp>
# include <cppad/configure.hpp>
# include <fstream>

# if CPPAD_HAS_MKSTEMP
# include <stdlib.h>
# include <unistd.h>
# else
# if CPPAD_HAS_TMPNAM_S
# include <stdio.h>
# else
# include <stdlib.h>
# endif
# endif


namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*!
Set check_for_nan

\param value
new value for this flag.
*/
template <class Base, class RecBase>
void ADFun<Base,RecBase>::check_for_nan(bool value)
{   check_for_nan_ = value; }

/*!
Get check_for_nan

\return
current value of check_for_nan_.
*/
template <class Base, class RecBase>
bool ADFun<Base,RecBase>::check_for_nan(void) const
{   return check_for_nan_; }

/*!
Stores a vector in a file when nans occur.

\param vec [in]
is the vector that is stored.

\param [out] file_name
is the file where the vector is stored
*/
template <class Base>
void put_check_for_nan(const CppAD::vector<Base>& vec, std::string& file_name)
{
    size_t char_size = sizeof(Base) * vec.size();
    // 2DO: add vec.data() to C11 tests and use it when C11 true
    // const char* char_ptr   = reinterpret_cast<const char*>( vec.data() );
    const char* char_ptr   = reinterpret_cast<const char*>( &vec[0] );

# if CPPAD_HAS_MKSTEMP
    char pattern[] = "/tmp/fileXXXXXX";
    int fd = mkstemp(pattern);
    file_name = pattern;
    ssize_t flag = write(fd, char_ptr, char_size);
    if( flag < 0 )
    {   std::cerr << "put_check_nan: write error\n";
        std::exit(1);
    }
    close(fd);
# else
# if CPPAD_HAS_TMPNAM_S
        std::vector<char> name(L_tmpnam_s);
        // if( tmpnam_s( name.data(), L_tmpnam_s ) != 0 )
        if( tmpnam_s( &name[0], L_tmpnam_s ) != 0 )
        {   CPPAD_ASSERT_KNOWN(
                false,
                "Cannot create a temporary file name"
            );
        }
        // file_name = name.data();
        file_name = &name[0];
# else
        file_name = tmpnam( nullptr );
# endif
    std::fstream file_out(file_name.c_str(), std::ios::out|std::ios::binary );
    file_out.write(char_ptr, char_size);
    file_out.close();
# endif
    return;
}

/*!
Gets a vector that was stored by put_check_for_nan.

\param vec [out]
is the vector that is stored.

\param file_name [in]
is the file where the vector is stored
*/
template <class Base>
void get_check_for_nan(CppAD::vector<Base>& vec, const std::string& file_name)
{   //
    std::streamsize char_size = std::streamsize( sizeof(Base) * vec.size() );
    // char* char_ptr   = reinterpret_cast<char*>( vec.data() );
    char* char_ptr   = reinterpret_cast<char*>( &vec[0] );
    //
    std::fstream file_in(file_name.c_str(), std::ios::in|std::ios::binary );
    file_in.read(char_ptr, char_size);
    //
    return;
}

} // END_CPPAD_NAMESPACE

# endif
