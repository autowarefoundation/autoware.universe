# ifndef CPPAD_LOCAL_UTILITY_CPPAD_VECTOR_ITR_HPP
# define CPPAD_LOCAL_UTILITY_CPPAD_VECTOR_ITR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cstddef>
# include <cppad/core/cppad_assert.hpp>
/*
------------------------------------------------------------------------------
$begin cppad_vector_itr_define$$
$spell
    Iterator
    cppad
    itr
    undef
    const
    endif
    hpp
$$

$section Vector Class Iterator Preprocessor Definitions$$

$head Syntax$$
$codep
# define CPPAD_CONST 0
# include <cppad/local/utility/cppad_vector_itr.hpp>
# undef CPPAD_LOCAL_UTILITY_CPPAD_VECTOR_ITR_HPP
# define CPPAD_CONST 1
# include <cppad/local/utility/cppad_vector_itr.hpp>
%$$

$head Beginning of cppad_vector_itr.hpp$$
The following preprocessor definition appears at the beginning of
$code cppad_vector_itr.hpp$$ and is used for the class definition in this file:
$codep
# if CPPAD_CONST
# define CPPAD_VECTOR_ITR const_cppad_vector_itr
# else
# define CPPAD_VECTOR_ITR cppad_vector_itr
# endif
$$

$head End of cppad_vector_itr.hpp$$
The following preprocessor definition appears at the end of
$code cppad_vector_itr.hpp$$ so that it can be included with a different
value for $code CPPAD_CONST$$:
$codep
# undef CPPAD_CONST
# undef CPPAD_VECTOR_ITR
$$

$end
*/
# if CPPAD_CONST
# define CPPAD_VECTOR_ITR const_cppad_vector_itr
# else
# define CPPAD_VECTOR_ITR cppad_vector_itr
# endif


// BEGIN_CPPAD_LOCAL_UTILITY_NAMESPACE
namespace CppAD { namespace local { namespace utility {

// so can be declared friend in cppad_vector_itr<Type>
template <class Type> class const_cppad_vector_itr;

// ==========================================================================
template <class Type> class CPPAD_VECTOR_ITR {
// ==========================================================================
/*
-----------------------------------------------------------------------------
$begin cppad_vector_itr_traits$$
$spell
    Iterator
$$

$section Vector Class Iterator Traits and Friends$$

$srccode%hpp% */
# if ! CPPAD_CONST
    friend class const_cppad_vector_itr<Type>;
# endif
public:
    typedef std::random_access_iterator_tag    iterator_category;
    typedef Type                               value_type;
    typedef std::ptrdiff_t                     difference_type;
    typedef Type*                              pointer;
    typedef Type&                              reference;
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_itr_ctor$$
$spell
    Iterator
    ptr
    cppad
    Namespace
    CppAD
    const
    iterators
    itr
$$

$section Vector Class Iterator Member Data and Constructors$$

$head Constructors$$

$subhead Constant$$
$codei%const_cppad_vector_itr %itr%()
%$$
$codei%const_cppad_vector_itr %itr%(%data%, %length%, %index%)
%$$
$codei%const_cppad_vector_itr %itr%(%other%)
%$$
$codei%const_cppad_vector_itr %itr%(%non_const_other%)
%$$

$subhead Not Constant$$
$codei%cppad_vector_itr %itr%()
%$$
$codei%cppad_vector_itr %itr%(%data%, %length%, %index%)
%$$
$codei%cppad_vector_itr %itr%(%other%)
%$$

$head Namespace$$
These definitions are in the $code CppAD::local::utility$$ namespace.

$head Indirection$$
We use an extra level of indirection in this routine so that
the iterator has the same values as the vector even if the vector changes.

$head data_$$
is a pointer to a constant pointer to data for this vector
(used by operations that are not supported by constant iterators).

$head length_$$
is a pointer to the length of the corresponding vector.

$head index_$$
is the current vector index corresponding to this iterator.

$head check_element$$
generates an assert with a known cause when the $code index_$$
does not correspond go a valid element and
$code NDEBUG$$ is not defined.

$head check_cop$$
Generates an assert with a known cause when the $code data_$$
for this vector is different from the other vector and
$code NDEBUG$$ is not defined.
This should be used by operators that compare iterators.


$head Source$$
$srccode%hpp% */
private:
# if CPPAD_CONST
    const Type* const* data_;
# else
    Type* const*       data_;
# endif
    const size_t*      length_;
    difference_type    index_;
    void check_element(void) const CPPAD_NDEBUG_NOEXCEPT
    {   CPPAD_ASSERT_KNOWN( 0 <= index_ && size_t(index_) < *length_,
            "CppAD vector iterator: accessing element out of range"
        );
    }
    void check_cop(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   CPPAD_ASSERT_KNOWN( data_ == other.data_,
            "CppAD vector iterator: comparing indices from different vectors"
        );
    }
public:
    CPPAD_VECTOR_ITR(void) noexcept
    : data_(nullptr), length_(nullptr), index_(0)
    { }
# if CPPAD_CONST
    const_cppad_vector_itr(
        const Type* const* data, const size_t* length, difference_type index
    ) noexcept
    : data_(data), length_(length), index_( difference_type(index) )
    { }
    // ctor a const_iterator from an iterator
    const_cppad_vector_itr(
        const cppad_vector_itr<Type>& non_const_other
    ) noexcept
    {   data_       = non_const_other.data_;
        length_     = non_const_other.length_;
        index_      = non_const_other.index_;
    }
# else
    cppad_vector_itr(
        Type* const* data, const size_t* length, difference_type index
    ) noexcept
    : data_(data), length_(length), index_( difference_type(index) )
    { }
# endif
    void operator=(const CPPAD_VECTOR_ITR& other) noexcept
    {   data_       = other.data_;
        length_     = other.length_;
        index_      = other.index_;
    }
    CPPAD_VECTOR_ITR(const CPPAD_VECTOR_ITR& other) noexcept
    {   *this = other; }
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_itr_inc$$
$spell
    Iterator
    itr
$$

$section Vector Class Iterator Increment Operators$$

$head Syntax$$
$codei%++%itr%
%$$
$codei%--%itr%
%$$
$icode%itr%++
%$$
$icode%itr%--
%$$

$head Source$$
$srccode%hpp% */
public:
    CPPAD_VECTOR_ITR& operator++(void) noexcept
    {   ++index_;
        return *this;
    }
    CPPAD_VECTOR_ITR& operator--(void) noexcept
    {   --index_;
        return *this;
    }
    CPPAD_VECTOR_ITR operator++(int) noexcept
    {   CPPAD_VECTOR_ITR ret(*this);
        ++index_;
        return ret;
    }
    CPPAD_VECTOR_ITR operator--(int) noexcept
    {   CPPAD_VECTOR_ITR ret(*this);
        --index_;
        return ret;
    }
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_itr_equal$$
$spell
    itr
    Iterator
$$

$section Vector Class Iterator Equality Operators$$
$spell
    iterators
$$

$head Syntax$$
$icode%itr% == %other%
%$$
$icode%itr% != %other%
%$$

$head Restrictions$$
It is an error to compare iterators corresponding to different
$code data_$$ vectors

$head Source$$
$srccode%hpp% */
public:
    bool operator==(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   check_cop(other);
        return index_ == other.index_;
    }
    bool operator!=(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   check_cop(other);
        return index_ != other.index_;
    }
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_itr_element$$
$spell
    itr
    Iterator
$$

$section Vector Class Iterator Access Elements$$

$head Syntax$$
$icode%element% = *%itr%
%$$
$codei%*%itr% = %element%
%$$
$icode%element% = %itr%[%n%]
%$$
$icode%itr%[%n%] = %element%
%$$

$head Source$$
$srccode%hpp% */
public:
# if CPPAD_CONST
    const Type& operator*(void) const
    {   check_element();
        return (*data_)[index_];
    }
    const Type& operator[](difference_type n)
    {   return *(*this + n);
    }
    const Type& operator[](size_t n)
    {   return *( *this + difference_type(n) );
    }
# else
    Type& operator*(void)
    {   check_element();
        return (*data_)[index_];
    }
    Type& operator[](difference_type n)
    {   return *(*this + n);
    }
    Type& operator[](size_t n)
    {   return *( *this + difference_type(n) );
    }
# endif
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_itr_random$$
$spell
    itr
    Iterator
    bool
    iterators
$$

$section Vector Class Iterator Random Access$$

$head Syntax$$
$icode%itr% %+-% = %n%
%$$
$icode%itr% = %other% %+-% %n%
%$$
$icode%itr% = %n% %+-% %other%
%$$
$icode%n% = %itr% - %other%
%$$
$code%b% = %itr% %cop% %other%
%$$

$subhead +-$$
The notation $icode +-$$ above is either $code +$$ or $code -$$.

$subhead cop$$
is one of the following:
$code <$$, $code <=$$,
$code >$$, $code >=$$.

$head itr, other$$
are iterators of the same type.

$head n$$
is a $code difference_type$$ object.

$head b$$
is a $code bool$$.

$head Restrictions$$
It is an error to use a $icode cop$$ with iterators corresponding to different
$code data_$$ vectors

$head Source$$
$srccode%hpp% */
public:
    // sum and difference operators
    CPPAD_VECTOR_ITR& operator+=(difference_type n) noexcept
    {   index_ += n;
        return *this;
    }
    CPPAD_VECTOR_ITR& operator-=(difference_type n) noexcept
    {   index_ -= n;
        return *this;
    }
    CPPAD_VECTOR_ITR operator+(difference_type n) const noexcept
    {   return CPPAD_VECTOR_ITR(data_, length_, index_ + n);
    }
    CPPAD_VECTOR_ITR operator-(difference_type n) const noexcept
    {   return CPPAD_VECTOR_ITR(data_, length_, index_ - n);
    }
    difference_type  operator-(const CPPAD_VECTOR_ITR& other) const
    noexcept
    {   return index_ - other.index_;
    }
    // comparison operators
    bool operator<(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   check_cop(other);
        return index_ < other.index_;
    }
    bool operator<=(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   check_cop(other);
        return index_ <= other.index_;
    }
    bool operator>(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   check_cop(other);
        return index_ > other.index_;
    }
    bool operator>=(const CPPAD_VECTOR_ITR& other) const CPPAD_NDEBUG_NOEXCEPT
    {   check_cop(other);
        return index_ >= other.index_;
    }
/* %$$
$srcthisfile%
    0%// BEGIN_BINARY_OP%// END_BINARY_OP%1
%$$
$end
*/
// ==========================================================================
}; // END_TEMPLATE_CLASS_CPPAD_VECTOR_ITR
// ==========================================================================

// BEGIN_BINARY_OP
template <class Type> CPPAD_VECTOR_ITR<Type> operator+(
    typename CPPAD_VECTOR_ITR<Type>::difference_type n  ,
    const CPPAD_VECTOR_ITR<Type>&               other   ) noexcept
{   return
    CPPAD_VECTOR_ITR<Type>(other.data_, other.length_, n + other.index_ );
}
template <class Type> CPPAD_VECTOR_ITR<Type> operator-(
    typename CPPAD_VECTOR_ITR<Type>::difference_type n  ,
    const CPPAD_VECTOR_ITR<Type>&               other   ) noexcept
{   return
    CPPAD_VECTOR_ITR<Type>(other.data_, other.length_, n - other.index_ );
}
// END_BINARY_OP

} } } // END_CPPAD_LOCAL_UTILITY_NAMESPACE

# undef CPPAD_CONST
# undef CPPAD_VECTOR_ITR
# endif
