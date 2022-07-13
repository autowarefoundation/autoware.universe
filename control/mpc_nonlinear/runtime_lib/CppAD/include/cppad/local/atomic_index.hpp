# ifndef CPPAD_LOCAL_ATOMIC_INDEX_HPP
# define CPPAD_LOCAL_ATOMIC_INDEX_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
$begin atomic_index$$
$spell
    ptr
$$

$section Store and Retrieve Atomic Function Information by Index$$

$head Syntax$$
$icode%index_out% = local::atomic_index<%Base%>(
    %set_null%, %index_in%, %type%, %name%, %ptr%
)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_ATOMIC_INDEX%// END_PROTOTYPE%1
%$$

$head Base$$
Is the base type for the tape for the atomic functions
that we are using an index to identify.

$head Special Case$$
In the special case,
$icode set_null$$ is true and $icode index_in$$ is zero.
For this case, $icode index_out$$ is set to
the number of atomic functions stored in $codei%atomic_index<%Base%>%$$
and no information is stored or changed.
In this case, the atomic functions correspond to $icode index_in$$ from
one to $icode index_out$$ inclusive.

$head set_null$$
If this is not the special case:
This value should only be true during a call to an atomic function destructor.
If it is true, the $icode ptr$$ corresponding to $icode index_in$$
is set to null.

$head index_in$$
If this is not the special case:

$subhead zero$$
The value $icode index_in$$ should only be zero
during a call to an atomic function constructor.
In this case, a copy of the input value of
$icode type$$, $codei%*%name%$$, and $icode ptr$$ are stored.
The value $icode index_out$$
is the $icode index_in$$ value corresponding to these input values.

$subhead non-zero$$
If $icode index_in$$ is non-zero,
the information corresponding to this index is returned.

$head type$$
If this is not the special case:
If $icode index_in$$ is zero, $icode type$$ is an input.
Otherwise it is set to the value corresponding to this index.
The type corresponding to an index
is intended to be $code 2$$ for $cref atomic_two$$ functions
and $code 3$$ for $cref atomic_three$$ functions.

$head name$$
If this is not the special case:
If $icode index_in$$ is zero, $code name$$ is an input and must not be null.
Otherwise, if $icode name$$ is not null, $codei%*%name%$$
is set to the name corresponding to $icode index_in$$.
Allowing for $icode name$$ to be null avoids
a string copy when it is not needed.

$head ptr$$
If this is not the special case:
If $icode index_in$$ is zero, $icode ptr$$ is an input.
Otherwise it is set to the value corresponding to $icode index_in$$.
In the special case where $icode set_null$$ is true,
$icode ptr$$ is set to the null pointer and this is the $icode ptr$$ value
corresponding to $icode index_in$$ for future calls to $code atomic_index$$.

$head index_out$$
If this is not the special case:
If $icode index_in$$ is zero,
$icode index_out$$ is non-zero and is the index value
corresponding to the input values for
$icode type$$, $codei%*%name%$$, and $icode ptr$$.
Otherwise, $index_out$$ is zero.

$end
*/
# include <vector>
# include <cppad/utility/thread_alloc.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE

struct atomic_index_info {
    size_t      type;
    std::string name;
    void*       ptr;
};

// BEGIN_ATOMIC_INDEX
template <class Base>
size_t atomic_index(
    bool               set_null      ,
    const size_t&      index_in      ,
    size_t&            type          ,
    std::string*       name          ,
    void*&             ptr           )
// END_PROTOTYPE
{   //
    // information for each index
    static std::vector<atomic_index_info> vec;
# ifndef NDEBUG
    if( index_in == 0 || set_null )
    {   CPPAD_ASSERT_KNOWN( ! thread_alloc::in_parallel(),
        "calling atomic function constructor or destructor in parallel mode"
        );
    }
# endif
    if( set_null & (index_in == 0) )
        return vec.size();
    //
    // case were we are retreving informaiton for an atomic function
    if( 0 < index_in )
    {   CPPAD_ASSERT_UNKNOWN( index_in <= vec.size() )
        //
        // case where we are setting the pointer to null
        if( set_null )
            vec[index_in-1].ptr = nullptr;
        //
        atomic_index_info& entry = vec[index_in - 1];
        type = entry.type;
        ptr  = entry.ptr;
        if( name != nullptr )
            *name  = entry.name;
        return 0;
    }
    //
    // case where we are storing information for an atomic function
    atomic_index_info entry;
    entry.type = type;
    entry.name = *name;
    entry.ptr  = ptr;
    vec.push_back(entry);
    //
    return vec.size();
}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
