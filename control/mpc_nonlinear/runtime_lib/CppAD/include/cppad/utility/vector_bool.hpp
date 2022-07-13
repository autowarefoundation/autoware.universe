# ifndef CPPAD_UTILITY_VECTOR_BOOL_HPP
# define CPPAD_UTILITY_VECTOR_BOOL_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


# include <cstddef>
# include <iostream>
# include <limits>
# include <cppad/core/cppad_assert.hpp>
# include <cppad/utility/check_simple_vector.hpp>
# include <cppad/utility/thread_alloc.hpp>
# include <cppad/local/utility/vector_bool.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// ============================================================================
class vectorBool {
// ============================================================================
/*
$begin vector_bool_member$$
$spell
    Bool
    vec
$$

$section vectorBool: Member Data$$

$head Syntax$$
$icode%vec%.unit_min()
%$$
$icode%vec%.bit_per_unit()
%$$


$head unit_t$$
Type used to pack multiple boolean (bit) values into one unit.
Logical operations are preformed one unit at a time.

$head bit_per_unit_$$
number of bits packed into each unit value in $code data_$$.

$head n_unit_$$
Number of unit values in $code data_$$.

$head length_$$
number of bits currently stored in this vector.

$head data_$$
pointer to where the bits are stored.

$head unit_min$$
minimum number of $code unit_t$$ values that can store $code length_$$ bits.
Note that this is really a function of $code length_$$.

$head size$$
is the number of boolean elements in the vector.

$head capacity$$
is the maximum number of boolean elements that will fit in the
current allocation for $code data_$$.

$head Source$$
$srccode%hpp% */
private:
    typedef size_t unit_t;
    static const size_t bit_per_unit_ = std::numeric_limits<unit_t>::digits;
    size_t    n_unit_;
    size_t    length_;
    unit_t    *data_;
    //
    size_t unit_min(void) const
    {   if( length_ == 0 )
            return 0;
        return (length_ - 1) / bit_per_unit_ + 1;
    }
public:
    static size_t bit_per_unit(void)
    {   return bit_per_unit_; }
    size_t size(void) const
    {   return length_; }
    size_t capacity(void) const
    {   return n_unit_ * bit_per_unit_; }
/* %$$
$end
-------------------------------------------------------------------------------
$begin vector_bool_typedef$$
$spell
    vec
    Bool
    const
$$

$section vectorBool Type Definitions$$

$head value_type$$
type corresponding to the elements of this vector
(note that non-const elements actually use
$cref/vectorBoolElement/vector_bool_element/$$).

$head Source$$
$srccode%hpp% */
public:
    typedef bool value_type;
/* %$$
$end
----------------------------------------------------------------------------
$begin vector_bool_ctor$$
$spell
    Bool
    vec
    alloc
$$
$section vectorBool: Constructors and Destructor$$

$head Default$$
$codei%vectorBool %vec%
%$$
creates an empty vector with no elements and $code n_unit_$$ zero.

$head Sizing$$
$codei%vectorBool %vec%(%n%)
%$$
where $icode n$$ is a $code size_t$$,
creates the vector $icode vec$$ with $icode n$$ elements and $code n_unit_$$
greater than or equal $code unit_min()$$.

$head Copy$$
$codei%vector<%Type%> %vec%(%other%)
%$$
where $icode other$$ is a $codei%vector<%Type%>%$$,
creates the vector $icode vec$$
with $icode%n% = %other%.size()%$$ elements and $code n_unit_$$
greater than or equal $code unit_min()$$.

$head Destructor$$
If $code n_unit_$$ is non-zero, the memory corresponding to data_
is returned to thread_alloc.

$head Source$$
$srccode%hpp%:
*/
    vectorBool(void) : n_unit_(0), length_(0), data_(nullptr)
    { }
    vectorBool(size_t n) : n_unit_(0), length_(n), data_(nullptr)
    {   resize(n); }
    vectorBool(const vectorBool& other)
    : n_unit_(0), length_(0), data_(nullptr)
    {   resize(other.length_);
        size_t n_used = unit_min();
        CPPAD_ASSERT_UNKNOWN( n_used <= n_unit_ );
        for(size_t i = 0; i < n_used; ++i)
            data_[i] = other.data_[i];
    }
    // n_unit_ is the only value necessary to make destructor work
    // for other after this move semantics constructor
    vectorBool(vectorBool&& other)
    : n_unit_(0), length_(0), data_(nullptr)
    {   swap(other); }
    ~vectorBool(void)
    {   clear(); }
/* %$$
$end
-----------------------------------------------------------------------------
$begin vector_bool_size$$
$spell
    Bool
    resize
    vec
$$

$section vectorBool: Change Size$$

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
$icode%vec%.n_unit_%$$ times $icode%vec%.bit_per_unit_%$$,
the only change is that $icode%vec%.length_%$$ is set to $icode n$$.
Otherwise the old elements are freed and a new vector is created
with $icode%vec%.length_%$$ equal to $icode n$$.

$head clear$$
the memory allocated for this vector is freed and
$icode%vec.length_%$$ and $icode%vec%.n_unit_%$$ are set to zero.

$end
------------------------------------------------------------------------------
*/
// BEGIN_RESIZE
public:
    void resize(size_t n)
// END_RESIZE
    {   length_ = n;
        // check if we can use the current memory
        size_t min_unit = unit_min();
        if( n_unit_ >= min_unit )
            return;
        // check if there is old memory to be freed
        if( n_unit_ > 0 )
        {   void* v_ptr = reinterpret_cast<void*>(data_);
            thread_alloc::return_memory(v_ptr);
        }
        // get new memory and set n_unit
        size_t min_bytes = min_unit * sizeof(unit_t);
        size_t cap_bytes;
        void* v_ptr = thread_alloc::get_memory(min_bytes, cap_bytes);
        data_       = reinterpret_cast<unit_t*>(v_ptr);
        n_unit_     = cap_bytes / sizeof(unit_t);
        CPPAD_ASSERT_UNKNOWN( n_unit_ >= min_unit );
    }
// BEGIN_CLEAR
    void clear(void)
// END_CLEAR
    {
        // check if there is old memory to be freed
        if( n_unit_ > 0 )
        {   void* v_ptr = reinterpret_cast<void*>(data_);
            thread_alloc::return_memory(v_ptr);
        }
        length_ = 0;
        n_unit_ = 0;
    }
/*
-------------------------------------------------------------------------------
$begin vector_bool_assign$$
$spell
    Bool
    resize
    vec
$$

$section vectorBool: Assignment Operators$$

$head Syntax$$
$icode%vec%.swap(%other%)
%$$
$icode%vec% = %other%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN_SWAP%// END_SWAP%1
%$$
$srcthisfile%
    0%// BEGIN_ASSIGN%// END_ASSIGN%1
%$$
$srcthisfile%
    0%// BEGIN_MOVE_SEMANTICS%// END_MOVE_SEMANTICS%1
%$$

$head swap$$
Swaps $code n_unit_$$, $code length_$$ and $code data_$$
between $icode vec$$ and $icode other$$.

$head Assignment$$
If the input value of $icode%vec%.length_%$$ is zero,
$cref/resize/vector_bool_size/resize/$$ is used to change its size to
be the same as other.
The size of $icode vec$$ and $icode other$$ are then compared and if
different, an assert with a know cause is generated.
The elements of $icode vec$$ are then individually assigned
to have the value of the corresponding elements of $icode other$$.

$head Move Semantics$$
A move semantics version of the assignment operator, implemented using
$code swap$$, is defined.

$end
-------------------------------------------------------------------------------
*/
// BEGIN_SWAP
    void swap(vectorBool& other)
// END_SWAP
    {   // swap with self case
       if( this == &other )
            return;
        std::swap(n_unit_,   other.n_unit_   );
        std::swap(length_,   other.length_   );
        std::swap(data_,     other.data_     );
        return;
    }
// BEGIN_ASSIGN
    vectorBool& operator=(const vectorBool& other)
// END_ASSIGN
    {   // If original length is zero, then resize it to other.
        // Otherwise a length mismatch is an error.
        if( length_ == 0 )
            resize( other.length_ );
        CPPAD_ASSERT_KNOWN(
            length_ == other.length_ ,
            "vectorBool: size miss match in assignment operation"
        );
        size_t n_used = unit_min();
        CPPAD_ASSERT_UNKNOWN( n_used <= n_unit_ );
        for(size_t i = 0; i < n_used; i++)
            data_[i] = other.data_[i];
        return *this;
    }
// BEGIN_MOVE_SEMANTICS
    vectorBool& operator=(vectorBool&& other)
// END_MOVE_SEMANTICS
    {   CPPAD_ASSERT_KNOWN(
            length_ == other.length_ || (length_ == 0),
            "vectorBool: size miss match in assignment operation"
        );
        swap(other);
        return *this;
    }
/*
-------------------------------------------------------------------------------
$begin vector_bool_subscript$$
$spell
    vec
    Bool
    const
$$

$section vectorBool: Subscript Operator$$

$head Syntax$$
$icode%target% = %vec%[%i%]
%$$
$icode%vec%[%i%] = %source%
%$$

$head target$$
In this syntax $icode vec$$ is $code const$$
and the value $icode%vec%[%i%]%$$ is a $code bool$$.

$head source$$
In this syntax $icode vec$$ is not $code const$$
and the value $icode%vec%[%i%]%$$ is a
$cref/vectorBoolElement/vector_bool_element/$$.

$head Source$$
$srccode%hpp% */
    bool operator[](size_t i) const
    {   CPPAD_ASSERT_KNOWN( i < length_,
            "vectorBool: index greater than or equal vector size"
        );
        size_t unit_index   = i / bit_per_unit_;
        size_t bit_index    = i - unit_index * bit_per_unit_;
        unit_t unit         = data_[unit_index];
        unit_t mask         = unit_t(1) << bit_index;
        return (unit & mask) != 0;
    }
    local::utility::vectorBoolElement operator[](size_t i)
    {   CPPAD_ASSERT_KNOWN( i < length_,
            "vectorBool: index greater than or equal vector size"
        );
        size_t unit_index   = i / bit_per_unit_;
        size_t bit_index    = i - unit_index * bit_per_unit_;
        unit_t mask         = unit_t(1) << bit_index;
        return local::utility::vectorBoolElement(data_ + unit_index , mask);
    }
/* %$$
$end
-------------------------------------------------------------------------------
$begin vector_bool_push_back$$
$spell
    Bool
    vec
$$

$section vectorBool: push_back$$

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
    void push_back(bool element)
// END_PUSH_BACK
    {   CPPAD_ASSERT_UNKNOWN( unit_min() <= n_unit_ );
        size_t old_length = length_;
        if( length_ + 1 > n_unit_ * bit_per_unit_ )
        {   CPPAD_ASSERT_UNKNOWN( unit_min() == n_unit_ );

            // create new vector with requuired size
            vectorBool vec(length_ + 1);

            // copy old data values
            size_t n_used = unit_min();
            CPPAD_ASSERT_UNKNOWN( n_used <= n_unit_ );
            for(size_t i = 0; i < n_used; ++i)
                vec.data_[i] = data_[i];

            // swap old and new vectors
            swap(vec);
        }
        else
            ++length_;
        CPPAD_ASSERT_UNKNOWN( length_ <= n_unit_ * bit_per_unit_ )
        size_t   unit_index = old_length / bit_per_unit_;
        size_t   bit_index  = old_length - unit_index * bit_per_unit_;
        unit_t mask         = unit_t(1) << bit_index;
        if( element )
            data_[unit_index] |= mask;
        else
            data_[unit_index] &= ~mask;
    }
/* %$$
$end
-------------------------------------------------------------------------------
$begin vector_bool_push_vector$$
$spell
    Bool
    vec
$$

$section vectorBool: push_vector$$

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
    {   CPPAD_ASSERT_UNKNOWN( unit_min() <= n_unit_ );
        CheckSimpleVector<bool, Vector>();
        size_t old_length = length_;
        size_t m           = other.size();
        if( length_ + m > n_unit_ * bit_per_unit_ )
        {
            // create new vector with requuired size
            vectorBool vec(length_ + m);

            // copy old data values
            size_t n_used = unit_min();
            CPPAD_ASSERT_UNKNOWN( n_used <= n_unit_ );
            for(size_t i = 0; i < n_used; ++i)
                vec.data_[i] = data_[i];

            // swap old and new vectors
            swap(vec);
        }
        else
            length_ += m;
        //
        // put the new elements in this vector
        CPPAD_ASSERT_UNKNOWN( length_ <= n_unit_ * bit_per_unit_ )
        for(size_t k = 0; k < m; k++)
        {   size_t unit_index = (old_length + k) / bit_per_unit_;
            size_t bit_index  = (old_length + k) - unit_index * bit_per_unit_;
            unit_t mask       = unit_t(1) << bit_index;
            if( other[k] )
                data_[unit_index] |= mask;
            else
                data_[unit_index] &= ~mask;
        }
    }
};

/*
$begin vector_bool_output$$
$spell
    Bool
    vec
$$

$section vectorBool: Output$$

$head Syntax$$
$icode%os% << vec%$$

$head Source$$
$srccode%hpp% */
inline std::ostream& operator << (std::ostream&  os , const vectorBool& vec )
{   for(size_t i = 0; i < vec.size(); ++i)
        os << vec[i];
    return os;
}
/* %$$
$end
*/

} // END_CPPAD_NAMESPACE
# endif
