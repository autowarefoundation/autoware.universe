# ifndef CPPAD_UTILITY_VECTOR_HPP
# define CPPAD_UTILITY_VECTOR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


# include <cstddef>
# include <iostream>
# include <cppad/core/cppad_assert.hpp>
# include <cppad/utility/check_simple_vector.hpp>
# include <cppad/utility/thread_alloc.hpp>

// Note that CPPAD_CONST is undefined by cppad_vector_itr.hpp
# define CPPAD_CONST 0
# include <cppad/local/utility/cppad_vector_itr.hpp>
# undef  CPPAD_LOCAL_UTILITY_CPPAD_VECTOR_ITR_HPP
# define CPPAD_CONST 1
# include <cppad/local/utility/cppad_vector_itr.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// ==========================================================================
template <class Type> class vector {
// ==========================================================================
/*
$begin cppad_vector_member$$
$spell
    vec
$$

$section Vector Class: Member Data$$

$head Syntax$$
$icode%vec%.capacity()
%$$
$icode%vec%.size()
%$$
$icode%vec%.data()
%$$

$head Type$$
is the type of the elements in the array.

$head capacity_$$
Number of $icode Type$$ elements in $code data_$$ that have been allocated
(and constructor has been called).

$head length_$$
Number of $icode Type$$ elements currently in this vector.

$head data_$$
Pointer to the first element of the vector
(not defined and should not be used when $code capacity_$$ is  0).

$head Source$$
$srccode%hpp% */
private:
    size_t capacity_;
    size_t length_;
    Type*  data_;
public:
    size_t capacity(void) const noexcept
    {   return capacity_; }
    size_t size(void) const noexcept
    {   return length_; }
    const Type* data(void) const noexcept
    {   return data_; }
    Type* data(void) noexcept
    {   return data_; }
/* %$$
$end
-----------------------------------------------------------------------------
$begin cppad_vector_typedef$$
$spell
    const_iterator
$$

$section Vector Class: Type Definitions$$

$head value_type$$
type corresponding to an element of a vector.

$head iterator$$
type corresponding to an iterator for a vector.

$head const_iterator$$
type corresponding to an iterator for a vector when
the vector is $code const$$.

$srccode%hpp% */
public:
    typedef Type                                         value_type;
    typedef local::utility::cppad_vector_itr<Type>       iterator;
    typedef local::utility::const_cppad_vector_itr<Type> const_iterator;
/* %$$
$end
-----------------------------------------------------------------------------
$begin cppad_vector_ctor$$
$spell
    vec
$$

$section Vector Class: Constructors and Destructor$$

$head Default$$
$codei%vector<%Type%> %vec%
%$$
creates an empty vector no elements and capacity zero.

$head Sizing$$
$codei%vector<%Type%> %vec%(%n%)
%$$
where $icode n$$ is a $code size_t$$ or $code int$$,
creates the vector $icode vec$$ with $icode n$$ elements and capacity
greater than or equal $icode n$$.

$head Copy$$
$codei%vector<%Type%> %vec%(%other%)
%$$
where $icode other$$ is a $codei%vector<%Type%>%$$,
creates the vector $icode vec$$
with $icode%n% = %other%.size()%$$ elements and capacity
greater than or equal $icode n$$.

$head Move Semantics$$
A move semantics version of the copy operator
is implemented using $code swap$$.

$head Destructor$$
If $code capacity_$$ is non-zero, call the destructor
for all the corresponding elements and then frees the corresponding memory.

$head delete_data$$
Call destructor and free all the allocated elements
(there are $code capacity_$$ such elements).

$head Source$$
$srccode%hpp% */
public:
    vector(void) noexcept
    : capacity_(0), length_(0), data_(nullptr)
    { }
    vector(size_t n) : capacity_(0), length_(0), data_(nullptr)
    {   resize(n); }
    vector(int n) : capacity_(0), length_(0), data_(nullptr)
    {   CPPAD_ASSERT_KNOWN(
            n >= 0,
            "CppAD::vector: attempt to create a vector with a negative size."
        );
        resize( size_t(n) );
    }
    vector(const vector& other) : capacity_(0), length_(0), data_(nullptr)
    {   resize(other.length_);
        for(size_t i = 0; i < length_; i++)
            data_[i] = other.data_[i];
    }
    // capacity_ is only value required to make destructor work for other
    // after this move semantics constuctor
    vector(vector&& other) : capacity_(0), length_(0), data_(nullptr)
    {   swap(other); }
    ~vector(void)
    {   if( capacity_ > 0 ) delete_data(data_); }
private:
    void delete_data(Type* data_ptr)
    {   thread_alloc::delete_array(data_ptr); }
/* %$$
$end
-----------------------------------------------------------------------------
$begin cppad_vector_size$$
$spell
    resize
    vec
$$

$section Vector Class: Change Size$$

$head Syntax$$
$icode%vec%.resize(%n%)
%$$
$icode%vec%.clear()%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_RESIZE%// END_RESIZE%1
%$$
$srcthisfile%
    0%// BEGIN_CLEAR%// END_CLEAR%1
%$$

$head n$$
is the number of elements in the new version of the vector.

$head resize$$
If $icode n$$ is less than or equal the input value of
$icode%vec%.capacity_%$$,
the only change is that $icode%vec%.length_%$$ is set to $icode n$$.
Otherwise, new memory is allocated for the vector and
$icode%vec%.length_%$$ elements are copied from the old vector
to the new one. I you do not need the old elements, you can first resize
to zero and then the $icode n$$ to avoid copying the elements.

$head clear$$
The destructor is called for all the elements of $icode vec$$
and then $icode%vec.length_%$$ and $icode%vec%.capacity_%$$ are set to zero.

$end
------------------------------------------------------------------------------
*/
// BEGIN_RESIZE
public:
    void resize(size_t n)
// END_RESIZE
    {   if( capacity_ < n )
        {   if( capacity_ == 0 )
            {   // get new memory and set capacity
                data_ = thread_alloc::create_array<Type>(n, capacity_);
            }
            else
            {   // save old information
                Type*  old_data     = data_;

                // get new memory and set capacity
                data_ = thread_alloc::create_array<Type>(n, capacity_);

                // copy old data
                for(size_t i = 0; i < length_; ++i)
                    data_[i] = old_data[i];

                // free old memory
                thread_alloc::delete_array(old_data);
            }
        }
        length_ = n;
    }
// BEGIN_CLEAR
    void clear(void)
// END_CLEAR
    {   length_ = 0;
        // check if there is old memory to be freed
        if( capacity_ > 0 )
            delete_data(data_);
        capacity_ = 0;
    }
/*
-------------------------------------------------------------------------------
$begin cppad_vector_assign$$
$spell
    resize
    vec
    cppad
$$

$section Vector Class: Assignment Operators$$

$head Syntax$$
$icode%vec%.swap(%other%)
%$$
$icode%vec% = %other%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_SWAP%// END_SWAP%1
%$$
$srcthisfile%
    0%// BEGIN_MOVE_ASSIGN%// END_MOVE_ASSIGN%1
%$$
$srcthisfile%
    0%// BEGIN_ASSIGN%// END_ASSIGN%1
%$$

$head swap$$
Swaps $code length_$$, $code capacity_$$ and $code data_$$
between $icode vec$$ and $icode other$$.

$head Assignment$$
see $cref/user API assignment/CppAD_vector/Assignment/$$

$head Move Semantics$$
The move semantics version of the assignment operator
is implemented using $code swap$$.

$end
-------------------------------------------------------------------------------
*/
// BEGIN_SWAP
public:
    // swap does not do any allocation and hence is declared noexcept
    void swap(vector& other) noexcept
// END_SWAP
    {  // special case where vec and other are the same vector
       if( this == &other )
            return;
        //
        std::swap(length_,   other.length_   );
        std::swap(capacity_, other.capacity_ );
        std::swap(data_,     other.data_     );
        return;
    }

// BEGIN_MOVE_ASSIGN
    // move assingment does not doe any allocation and hence is declared noexcept
    vector& operator=(vector&& other) noexcept
// END_MOVE_ASSIGN
    {   swap(other);
        return *this;
    }

// BEGIN_ASSIGN
    vector& operator=(const vector& other)
// END_ASSIGN
    {   // avoid copying old elements
        resize(0);
        // new size for this vector
        resize( other.length_ );
        // copy elements from other
        for(size_t i = 0; i < length_; i++)
            data_[i] = other.data_[i];
        return *this;
    }
/*
-------------------------------------------------------------------------------
$begin cppad_vector_subscript$$
$spell
    vec
$$

$section Vector Class: Subscript Operator$$

$head Syntax$$
$icode%element% = %vec%[%i%]
%$$
$icode%vec%[%i%] = %element%
%$$

$head Source$$
$srccode%hpp% */
    const Type& operator[]( size_t i) const
    {   CPPAD_ASSERT_KNOWN( i < length_,
            "vector: index greater than or equal vector size"
        );
        return data_[i];
    }
    Type& operator[](size_t i)
    {   CPPAD_ASSERT_KNOWN(i < length_,
            "vector: index greater than or equal vector size"
        );
        return data_[i];
    }
    template <class Index> const Type& operator[]( Index i) const
    {   return (*this)[size_t(i)]; }
    template <class Index> Type& operator[](Index i)
    {   return (*this)[size_t(i)]; }
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_push_back$$
$spell
    vec
$$

$section Vector Class: push_back$$

$head Syntax$$
$icode%vec%.push_back(%element%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PUSH_BACK%// END_PUSH_BACK%1
%$$

$head Documentation$$
see $cref/use API push_back/cppad_vector_push_back/$$

$end
*/
// BEGIN_PUSH_BACK
    void push_back(const Type& element)
// END_PUSH_BACK
    {   // case where no allocation is necessary
        if( length_ < capacity_ )
        {   data_[length_++] = element;
            return;
        }
        CPPAD_ASSERT_UNKNOWN( length_ == capacity_ );

        // create new vector with required size
        vector vec(length_ + 1);

        // copy old data values
        for(size_t i = 0; i < length_; ++i)
            vec.data_[i] = data_[i];

        // put the new element in the new vector
        CPPAD_ASSERT_UNKNOWN( vec.length_ == length_ + 1);
        vec.data_[length_] = element;

        // swap old and new vectors
        swap(vec);
    }
/* %$$
$end
-------------------------------------------------------------------------------
$begin cppad_vector_push_vector$$
$spell
    vec
$$

$section Vector Class: push_vector$$

$head Syntax$$
$icode%vec%.push_vector(%other%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_PUSH_VECTOR%// END_PUSH_VECTOR%1
%$$


$head Documentation$$
see $cref/use API push_vector/cppad_vector_push_vector/$$

$end
*/
// BEGIN_PUSH_VECTOR
    template <class Vector> void push_vector(const Vector& other)
// END_PUSH_VECTOR
    {   // can not use push_back because MS V++ 7.1 did not resolve
        // to non-template member function when scalar is used.
        //
        CheckSimpleVector<Type, Vector>();
        size_t m = other.size();

        // case where no allcoation is necessary
        if( length_ + m <= capacity_ )
        {   for(size_t i = 0; i < m; i++)
                data_[length_++] = other[i];
            return;
        }

        // create new vector with required size
        vector vec(length_ + m);

        // copy old data values
        for(size_t i = 0; i < length_; ++i)
            vec.data_[i] = data_[i];

        // put the new elements in the new vector
        CPPAD_ASSERT_UNKNOWN( vec.length_ == length_ + m );
        for(size_t i = 0; i < m; i++)
            vec.data_[length_ + i] = other[i];

        // swap old and new vectors
        swap(vec);
    }
/*
------------------------------------------------------------------------------
$begin cppad_vector_itr_fun$$
$spell
    vec
    iterator
$$

$section Vector Class: Iterator Functions$$

$head Syntax$$
$icode%os%vec%.begin()
%$$
$icode%os%vec%.end()
%$$

$head Source$$
$srccode%hpp% */
    const_iterator begin(void) const noexcept
    {    return const_iterator(&data_, &length_, 0); }
    const_iterator end(void) const noexcept
    {   typedef typename const_iterator::difference_type difference_type;
        difference_type index = static_cast<difference_type>(length_);
        return const_iterator(&data_, &length_, index);
    }
    //
    iterator begin(void) noexcept
    {    return iterator(&data_, &length_, 0); }
    iterator end(void) noexcept
    {   typedef typename iterator::difference_type difference_type;
        difference_type index = static_cast<difference_type>(length_);
        return iterator(&data_, &length_, index);
    }
/* %$$
$end
*/

// =========================================================================
};  // END_TEMPLATE_CLASS_VECTOR
// =========================================================================

/*
$begin cppad_vector_output$$
$spell
    vec
$$

$section Vector Class: Output$$

$head Syntax$$
$icode%os% << vec%$$

$head Source$$
$srccode%hpp% */
template <class Type>
std::ostream& operator << (std::ostream&  os , const CppAD::vector<Type>& vec )
{   os << "{ ";
    for(size_t i = 0; i < vec.size(); ++i)
    {   os << vec[i];
        if( i + 1 < vec.size() )
            os << ", ";
    }
    os << " }";
    return os;
}
/* %$$
$end
*/

} // END_CPPAD_NAMESPACE

// user API specifies that vector_bool.hpp is included by vector.hpp
# include <cppad/utility/vector_bool.hpp>

# endif
