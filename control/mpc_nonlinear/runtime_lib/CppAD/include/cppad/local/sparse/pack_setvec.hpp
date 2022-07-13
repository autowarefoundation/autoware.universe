# ifndef CPPAD_LOCAL_SPARSE_PACK_SETVEC_HPP
# define CPPAD_LOCAL_SPARSE_PACK_SETVEC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/core/cppad_assert.hpp>
# include <cppad/local/pod_vector.hpp>

// BEGIN_CPPAD_LOCAL_SPARSE_NAMESPACE
namespace CppAD { namespace local { namespace sparse {

// forward declaration of iterator class
class pack_setvec_const_iterator;

// ============================================================================
class pack_setvec {
// ============================================================================
/*
$begin pack_setvec_member_data$$
$spell
    setvec
    resize
$$

$section class pack_setvec: Private Member Data$$

$head Pack$$
Type used to pack multiple elements of a set (multiple bits) onto one
$icode Pack$$ value.

$head n_bit_$$
Number of bits (elements) per $icode Pack$$ value.

$head zero_$$
The $icode Pack$$ value with all bits zero.

$head one_$$
The $icode Pack$$ value with all bits zero, except for the lowest order bit.

$head n_set_$$
Number of sets that we are representing.

$head end_$$
The possible elements in each set are $code 0$$, $code 1$$, ...,
$code end_-1$$.

$head n_pack_$$
Number of Pack values used to represent one set in the vector; i.e.,
to represent $code end_$$ bits.

$head data_$$
Data for all of the sets.

$head Source Code$$
$srccode%hpp% */
private:
    typedef size_t    Pack;
    const size_t      n_bit_;
    const Pack        zero_;
    const Pack        one_;
    size_t            n_set_;
    size_t            end_;
    size_t            n_pack_;
    pod_vector<Pack>  data_;
/* %$$
$end
-----------------------------------------------------------------------------
$begin pack_setvec_vec_memory$$
$spell
    setvec
$$

$section class pack_setvec: Approximate Memory Used by Vector$$

$head Public$$
This function is declared public, but is not part of
$cref SetVector$$ concept.

$head Implementation$$
$srccode%hpp% */
public:
    size_t memory(void) const
    {   return data_.capacity() * sizeof(Pack); }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_vec_print$$
$spell
    setvec
$$

$section class pack_setvec: Print a Vector of Sets$$


$head Public$$
This function is declared public, but is not part of
$cref SetVector$$ concept.

$head Prototype$$
$srccode%hpp% */
public:
    void print(void) const;
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_iterators$$
$spell
    setvec
    Iterators
    typedef
    const_iterator
$$

$section class pack_setvec: Iterators$$

$head SetVector Concept$$
$cref/const_iterator/SetVector/const_iterator/$$

$head typedef$$
$srccode%hpp% */
public:
    /// declare a const iterator
    friend class pack_setvec_const_iterator;
    typedef pack_setvec_const_iterator const_iterator;
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_default_ctor$$
$spell
    setvec
$$

$section class pack_setvec: Default Constructor$$

$head SetVector Concept$$
$cref/constructor/SetVector/Vector Operations/Constructor/$$

$head n_bit_$$
This member variable is set to the number of bits in a $icode Pack$$ value.

$head one_$$
This member variable has only its lowest order bit non-zero;

$head data_$$
This member is initialized as the empty vector; i.e., size zero..

$head Other$$
All the other member data are $code size_t$$ values
that are initialized as zero.

$head Implementation$$
$srccode%hpp% */
public:
    pack_setvec(void) :
    n_bit_( std::numeric_limits<Pack>::digits ),
    zero_(0), one_(1), n_set_(0), end_(0), n_pack_(0), data_(0)
    { }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_destructor$$
$spell
    setvec
$$

$section class pack_setvec: Destructor$$

$head Implementation$$
$srccode%hpp% */
public:
    ~pack_setvec(void)
    { }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_copy_ctor$$
$spell
    setvec
    CppAD
$$

$section class pack_setvec: Copy Constructor$$

$head v$$
The vector of sets that we are attempting to make a copy of.

$head Implementation$$
Using the copy constructor is probably due to a $code pack_setvec$$
being passed by value instead of by reference.
This is a CppAD programing error (not CppAD user error).
$srccode%hpp% */
public:
    pack_setvec(const pack_setvec& v) :
    n_bit_( std::numeric_limits<Pack>::digits ), zero_(0), one_(1)
    {   CPPAD_ASSERT_UNKNOWN(0); }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_vec_resize$$
$spell
    setvec
    resize
$$

$section class pack_setvec: Vector resize$$

$head SetVector Concept$$
$cref/vector resize/SetVector/Vector Operations/resize/$$

$head Prototype$$
$srccode%hpp% */
public:
    void resize(size_t n_set, size_t end)
/* %$$
$end
*/
    {   n_set_          = n_set;
        end_            = end;
        if( n_set_ == 0 )
        {   CPPAD_ASSERT_UNKNOWN( end == 0 );
            data_.clear();
            return;
        }
        // now start a new vector with empty sets
        Pack zero(0);
        //
        n_pack_         = ( 1 + (end_ - 1) / n_bit_ );
        size_t i        = n_set_ * n_pack_;
        //
        data_.resize(i);
        while(i--)
            data_[i] = zero;
    }
/* %$$
-------------------------------------------------------------------------------
$begin pack_setvec_vec_n_set$$
$spell
    setvec
$$

$section class pack_setvec: Number of Sets$$

$head SetVector Concept$$
$cref/n_set/SetVector/Vector Operations/n_set/$$

$head Implementation$$
$srccode%hpp% */
public:
    size_t n_set(void) const
    {   return n_set_;  }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_vec_end$$
$spell
    setvec
$$

$section class pack_setvec: End Value$$

$head SetVector Concept$$
$cref/end/SetVector/Vector Operations/end/$$

$head Implementation$$
$srccode%hpp% */
public:
    size_t end(void) const
    {   return end_; }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_vec_assignment$$
$spell
    setvec
$$

$section class pack_setvec: Vector Assignment$$

$head SetVector Concept$$
$cref/vector assignment/SetVector/Vector Operations/Assignment/$$

$head Prototype$$
$srccode%hpp% */
public:
    void operator=(const pack_setvec& other)
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( n_bit_  == other.n_bit_);
        CPPAD_ASSERT_UNKNOWN( zero_   == other.zero_);
        CPPAD_ASSERT_UNKNOWN( one_    == other.one_);
        n_set_  = other.n_set_;
        end_    = other.end_;
        n_pack_ = other.n_pack_;
        data_   = other.data_;
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_vec_swap$$
$spell
    setvec
$$

$section class pack_setvec: Vector Swap$$

$head SetVector Concept$$
$cref/vector swap/SetVector/Vector Operations/swap/$$

$head Prototype$$
$srccode%hpp% */
public:
    void swap(pack_setvec& other)
/* %$$
$end
*/
    {   // size_t objects
        CPPAD_ASSERT_UNKNOWN( n_bit_  == other.n_bit_);
        CPPAD_ASSERT_UNKNOWN( zero_   == other.zero_);
        CPPAD_ASSERT_UNKNOWN( one_    == other.one_);
        std::swap(n_set_  , other.n_set_);
        std::swap(end_    , other.end_);
        std::swap(n_pack_ , other.n_pack_);
        //
        // pod_vectors
        data_.swap(other.data_);
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_number_elements$$
$spell
    setvec
$$

$section class pack_setvec: Number of Elements in a Set$$

$head SetVector Concept$$
$cref/number_elements/SetVector/number_elements/$$

$head Prototype$$
$srccode%hpp% */
public:
    size_t number_elements(size_t i) const
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( i < n_set_ );
        //
        // special case where data_[i] is 0 or 1
        if( end_ == 1 )
        {   CPPAD_ASSERT_UNKNOWN( n_pack_ == 1 );
            return size_t( data_[i] );
        }
        //
        // initialize count of non-zero bits in this set
        size_t count = 0;
        //
        // mask corresonding to first bit in Pack
        Pack mask = one_;
        //
        // number of bits in last Packing unit
        size_t n_last = (end_ - 1) % n_bit_ + 1;
        //
        // count bits in last unit
        Pack unit = data_[(i + 1) * n_pack_ - 1];
        for(size_t bit = 0; bit < n_last; ++bit)
        {   CPPAD_ASSERT_UNKNOWN( mask >= one_ );
            if( mask & unit )
                ++count;
            mask = mask << 1;
        }
        if( n_pack_ == 1 )
            return count;
        //
        // count bits in other units
        for(size_t bit = 0; bit < n_bit_; ++bit)
        {   CPPAD_ASSERT_UNKNOWN( mask >= one_ );
            size_t k = n_pack_;
            while(--k)
            {   if( data_[i * n_pack_ + k] & mask )
                    ++count;
            }
            mask = mask << 1;
        }
        return count;
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_add_element$$
$spell
    setvec
$$

$section class pack_setvec: Add an Elements to a Set$$

$head SetVector Concept$$
$cref/add_element/SetVector/add_element/$$

$head Prototype$$
$srccode%hpp% */
public:
    void add_element(size_t i, size_t element)
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( i   < n_set_ );
        CPPAD_ASSERT_UNKNOWN( element < end_ );
        if( end_ == 1 )
            data_[i] |= one_;
        else
        {   size_t j  = element / n_bit_;
            size_t k  = element - j * n_bit_;
            Pack mask = one_ << k;
            data_[ i * n_pack_ + j] |= mask;
        }
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_post_element$$
$spell
    setvec
$$

$section class pack_setvec: Add an Elements to a Set$$

$head SetVector Concept$$
$cref/post_element/SetVector/post_element/$$

$head Implementation$$
$srccode%hpp% */
public:
    void post_element(size_t i, size_t element)
    {   add_element(i, element); }
/* %$$
$end
*/
/*
-------------------------------------------------------------------------------
$begin pack_setvec_process_post$$
$spell
    setvec
$$

$section class pack_setvec: Add Posted Elements to a Set$$

$head SetVector Concept$$
$cref/process_post/SetVector/process_post/$$

$head Implementation$$
$srccode%hpp% */
public:
    void process_post(size_t i)
    {   return; }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_is_element$$
$spell
    setvec
$$

$section class pack_setvec: Is an Element in a Set$$

$head SetVector Concept$$
$cref/is_element/SetVector/is_element/$$

$head Prototype$$
$srccode%hpp% */
public:
    bool is_element(size_t i, size_t element) const
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( i   < n_set_ );
        CPPAD_ASSERT_UNKNOWN( element < end_ );
        if( end_ == 1 )
            return data_[i] != zero_;
        //
        size_t j  = element / n_bit_;
        size_t k  = element - j * n_bit_;
        Pack mask = one_ << k;
        return (data_[i * n_pack_ + j] & mask) != zero_;
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_clear$$
$spell
    setvec
$$

$section class pack_setvec: Assign a Set to be Empty$$

$head SetVector Concept$$
$cref/clear/SetVector/clear/$$

$head Prototype$$
$srccode%hpp% */
public:
    void clear(size_t target)
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( target < n_set_ );
        size_t t = target * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = zero_;
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_assignment$$
$spell
    setvec
$$

$section class pack_setvec: Assign a Set To Equal Another Set$$

$head SetVector Concept$$
$cref/assignment/SetVector/assignment/$$

$head Prototype$$
$srccode%hpp% */
public:
    void assignment(
        size_t               this_target  ,
        size_t               other_value  ,
        const pack_setvec&   other        )
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( this_target  <   n_set_        );
        CPPAD_ASSERT_UNKNOWN( other_value  <   other.n_set_  );
        CPPAD_ASSERT_UNKNOWN( n_pack_      ==  other.n_pack_ );
        size_t t = this_target * n_pack_;
        size_t v = other_value * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = other.data_[v++];
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_binary_union$$
$spell
    setvec
$$

$section class pack_setvec: Assign a Set To Equal Union of Two Sets$$

$head SetVector Concept$$
$cref/binary_union/SetVector/binary_union/$$

$head Prototype$$
$srccode%hpp% */
public:
    void binary_union(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const pack_setvec&      other        )
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( this_target < n_set_         );
        CPPAD_ASSERT_UNKNOWN( this_left   < n_set_         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.n_set_   );
        CPPAD_ASSERT_UNKNOWN( n_pack_    ==  other.n_pack_ );

        size_t t  = this_target * n_pack_;
        size_t l  = this_left  * n_pack_;
        size_t r  = other_right * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = ( data_[l++] | other.data_[r++] );
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_binary_intersection$$
$spell
    setvec
$$

$section class pack_setvec: Assign a Set To Intersection of Two Sets$$

$head SetVector Concept$$
$cref/binary_intersection/SetVector/binary_intersection/$$

$head Prototype$$
$srccode%hpp% */
public:
    void binary_intersection(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const pack_setvec&      other        )
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( this_target < n_set_         );
        CPPAD_ASSERT_UNKNOWN( this_left   < n_set_         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.n_set_   );
        CPPAD_ASSERT_UNKNOWN( n_pack_    ==  other.n_pack_ );

        size_t t  = this_target * n_pack_;
        size_t l  = this_left  * n_pack_;
        size_t r  = other_right * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = ( data_[l++] & other.data_[r++] );
    }
// ==========================================================================
}; // END_CLASS_PACK_SETVEC
// ==========================================================================


// =========================================================================
class pack_setvec_const_iterator { // BEGIN_CLASS_PACK_SETVEC_CONST_ITERATOR
// =========================================================================

/*
$begin pack_setvec_const_iterator_member_data$$
$spell
    setvec
    const_iterator
$$

$section class pack_setvec_const_iterator private: Member Data$$

$head Pack$$
This is the same type as
$cref/pack_setvec Pack/pack_setvec_member_data/Pack/$$.

$head n_bit_$$
This is a reference to
$cref/pack_setvec n_bit_/pack_setvec_member_data/n_bit_/$$.

$head one_$$
This is a reference to
$cref/pack_setvec one_/pack_setvec_member_data/one_/$$.

$head n_pack_$$
This is a reference to
$cref/pack_setvec n_pack_/pack_setvec_member_data/n_pack_/$$.

$head end_$$
This is a reference to
$cref/pack_setvec end_/pack_setvec_member_data/end_/$$.

$head data_$$
This is a reference to
$cref/pack_setvec data_/pack_setvec_member_data/data_/$$.

$head data_index_$$
Index in $code data_$$ where the next element is located.

$head next_element$$
Value of the next element in this set
If $code next_element_$$ equals $code end_$$,
no next element exists; i.e., past end of the set.

$head Source Code$$
$srccode%hpp% */
private:
    typedef pack_setvec::Pack Pack;
    const size_t&             n_bit_;
    const Pack&               one_;
    const size_t&             n_pack_;
    const size_t&             end_;
    const pod_vector<Pack>&   data_;
    size_t                    data_index_;
    size_t                    next_element_;
public:
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_const_iterator_ctor$$
$spell
    setvec
    const_iterator
$$

$section class pack_setvec_const_iterator: Constructor$$

$head SetVector Concept$$
$cref/iterator constructor/SetVector/const_iterator/Constructor/$$

$head Prototype$$
$srccode%hpp% */
public:
    pack_setvec_const_iterator (const pack_setvec& pack, size_t set_index)
/* %$$
$end
*/
    :
    n_bit_         ( pack.n_bit_ )        ,
    one_           ( pack.one_   )        ,
    n_pack_        ( pack.n_pack_ )       ,
    end_           ( pack.end_ )          ,
    data_          ( pack.data_ )         ,
    data_index_    ( set_index * n_pack_ )
    {   CPPAD_ASSERT_UNKNOWN( set_index < pack.n_set_ );
        CPPAD_ASSERT_UNKNOWN( 0 < end_ );
        //
        next_element_ = 0;
        if( data_[data_index_] & one_ )
            return;
        //
        // element with index zero is not in this set of integers,
        // advance to first element or end
        ++(*this);
    }
/*
-------------------------------------------------------------------------------
$begin pack_setvec_const_iterator_dereference$$
$spell
    setvec
    const_iterator
    Dereference
$$

$section class pack_setvec_const_iterator: Dereference$$

$head SetVector Concept$$
$cref/iterator deference/SetVector/const_iterator/Dereference/$$

$head Implementation$$
$srccode%hpp% */
    size_t operator*(void) const
    {   return next_element_; }
/* %$$
$end
-------------------------------------------------------------------------------
$begin pack_setvec_const_iterator_increment$$
$spell
    setvec
    const_iterator
$$

$section class pack_setvec_const_iterator: Increment$$

$head SetVector Concept$$
$cref/iterator increment/SetVector/const_iterator/Increment/$$

$head Prototype$$
$srccode%hpp% */
public:
    pack_setvec_const_iterator& operator++(void)
/* %$$
$end
*/
    {   CPPAD_ASSERT_UNKNOWN( next_element_ <= end_ );
        if( next_element_ == end_ )
            return *this;
        //
        ++next_element_;
        if( next_element_ == end_ )
            return *this;
        //
        // bit index corresponding to next element
        size_t bit = next_element_ % n_bit_;
        //
        // check if we have advanced to the next data index
        if( bit == 0 )
            ++data_index_;
        //
        // initialize mask
        size_t mask = one_ << bit;
        //
        while( next_element_ < end_ )
        {   // check if this element is in the set
            if( data_[data_index_] & mask )
                return *this;
            //
            // try next larger element
            ++next_element_;
            ++bit;
            mask <<= 1;
            //
            // check if we must go to next packed data index
            CPPAD_ASSERT_UNKNOWN( bit <= n_bit_ );
            if( bit == n_bit_ )
            {   // get next packed value
                bit   = 0;
                mask  = one_;
                ++data_index_;
            }
        }
        CPPAD_ASSERT_UNKNOWN( next_element_ == end_ );
        return *this;
    }
// =========================================================================
}; // END_CLASS_PACK_SETVEC_CONST_ITERATOR
// =========================================================================

// Implemented after pack_setvec_const_iterator so can use it
inline void pack_setvec::print(void) const
{   std::cout << "pack_setvec:\n";
    for(size_t i = 0; i < n_set(); i++)
    {   std::cout << "set[" << i << "] = {";
        const_iterator itr(*this, i);
        while( *itr != end() )
        {   std::cout << *itr;
            if( *(++itr) != end() )
                std::cout << ",";
        }
        std::cout << "}\n";
    }
    return;
}

// ----------------------------------------------------------------------------
/*
$begin sparsity_user2internal_pack_setvec$$
$spell
    setvec
    bool
$$

$section Copy A Boolean Sparsity Pattern To A pack_setvec Object$$

$head SetVector$$
is a $cref/simple vector/SimpleVector/$$ type with elements of type
$code bool$$ containing the input sparsity pattern.

$head  internal$$
The input value of this object does not matter.
Upon return it contains the same sparsity pattern as $icode user$$
(or its transpose).

$head user$$
is the sparsity pattern we are copying to $icode internal$$.

$head  n_set$$
is the number of sets in the output sparsity pattern $icode internal$$.

$head end$$
is the end value for the output sparsity pattern $icode internal$$.

$head transpose$$
If $icode transpose$$ is false,
element $icode j$$ is in the $th i$$ $icode internal$$ set if
$codei%
    %user%[ %i% * %end% + %j% ]
%$$
Otherwise,
element $icode j$$ is in the $th i$$ $icode internal$$ set if
$codei%
    %user%[ %i% * %n_set% + %j% ]
%$$

$head error_msg$$
is the error message to display if
$codei%
    %n_set% * %end% != %user%.size()
%$$

$head Prototype$$
$srccode%hpp% */
template<class SetVector>
void sparsity_user2internal(
    pack_setvec&            internal  ,
    const SetVector&        user      ,
    size_t                  n_set     ,
    size_t                  end       ,
    bool                    transpose ,
    const char*             error_msg )
/* %$$
$end
*/
{   CPPAD_ASSERT_KNOWN(size_t( user.size() ) == n_set * end, error_msg );

    // size of internal sparsity pattern
    internal.resize(n_set, end);

    if( transpose )
    {   // transposed pattern case
        for(size_t j = 0; j < end; j++)
        {   for(size_t i = 0; i < n_set; i++)
            {   // no advantage to using post_element for pack_setvec
                if( user[ j * n_set + i ] )
                    internal.add_element(i, j);
            }
        }
        return;
    }
    else
    {   for(size_t i = 0; i < n_set; i++)
        {   for(size_t j = 0; j < end; j++)
            {   // no advantage to using post_element for pack_setvec
                if( user[ i * end + j ] )
                    internal.add_element(i, j);
            }
        }
    }
    return;
}

} } } // END_CPPAD_LOCAL_SPARSE_NAMESPACE

# endif
