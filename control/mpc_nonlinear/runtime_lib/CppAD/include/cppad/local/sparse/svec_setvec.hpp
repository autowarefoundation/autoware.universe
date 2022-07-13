# ifndef CPPAD_LOCAL_SPARSE_SVEC_SETVEC_HPP
# define CPPAD_LOCAL_SPARSE_SVEC_SETVEC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/define.hpp>
# include <cppad/local/is_pod.hpp>
# include <list>

// BEGIN_CPPAD_LOCAL_SPARSE_NAMESPACE
namespace CppAD { namespace local { namespace sparse {

/*!
\file svec_setvec.hpp
Vector of sets of positive integers stored as size_t vector
with the element values strictly increasing.

Testing indicates this does not work as well as using sparse::list_setvec
(not currently being used except by test_more/general/local/vector_set.cpp).
*/
class svec_setvec_const_iterator;

// =========================================================================
/*!
Vector of sets of positive integers, each set stored as a size_t vector.

All the public members for this class are also in the
sparse::pack_setvec and sparse::list_setvec classes.
This defines the CppAD vector_of_sets concept.
*/
class svec_setvec {
    friend class svec_setvec_const_iterator;
private:
    /// Possible elements in each set are 0, 1, ..., end_ - 1;
    size_t end_;

    /// number of elements in data_ that have been allocated
    /// and are no longer being used.
    size_t data_not_used_;

    /// The data for all the singly linked lists.
    pod_vector<size_t> data_;

    /*!
    Starting point for i-th set is start_[i].

    \li
    If the i-th set has no elements, start_[i] is zero.
    Othersize the conditions below hold.

    \li
    data_[ start_[i] ] is the reference count for this set

    \li
    data_[ start_[i] + 1 ] is the first element in this set.

    \li
    data_[ start_[i] + 2 ] is the first element in this set.

    \li
    data_[ start_[i] + 2 + n ] = end_ where n is the number of
    elements in this set
    */
    pod_vector<size_t> start_;

    /*!
    Vectors of elements that have not yet been added to corresponding sets.

    \li
    If all the post_element calls for the i-th set have been added,
    post_[i] is zero. Otherwise the conditions below hold.

    \li
    data_[ post_[i] ]  the number of elements that have been posted,
    but not yet added, to set i.

    \li
    data_[ post_[i] + 1 ] is the capacity for holding elements
    which is greater than or equal the number of elements.

    \li
    data_[ post_[i] + 2 ] is the first element that has been posted,
    but not yet added, to set i.

    \li
    data_[ post_[i] + 1 + n] is the last element that has been posted,
    but not yet added, to set i.
    Here n is the number of elements that are posted and not yet added
    to set i.
    */
    pod_vector<size_t> post_;
    // -----------------------------------------------------------------
    /*!
    Counts references to a set.

    \param i
    is the index of the set that we are counting the references to.

    \return
    if the set is empty, the return value is zero.
    Otherwise it is the number of sets that share the same vector.
    */
    size_t reference_count(size_t i) const
    {   // start data index
        size_t start = start_[i];
        if( start == 0 )
            return 0;
        //
        // reference count
        return data_[start];
    }
    // -----------------------------------------------------------------
    /*!
    drop a set.

    \param i
    is the index of the set that will be dropped.

    \par reference_count
    if the set is non-empty,
    the reference count corresponding to index will be decremented.

    \return
    is the number of elements of data_ that will be lost when the set is
    dropped. This is non-zero when the initial reference count is one.
    */
    size_t drop(size_t i)
    {
        // reference count
        size_t ref_count = reference_count(i);

        // empty set case
        if( ref_count == 0 )
            return 0;

        // start
        size_t start = start_[i];
        CPPAD_ASSERT_UNKNOWN( data_[start] == ref_count );

        // decrement reference counter
        data_[start]--;

        // nothing lost unless new reference count is zero
        if( ref_count != 1 )
            return 0;

        // number of elements in the set
        size_t length = data_[start + 1];

        // reference count, length, end marker, plus elements in set
        size_t number_lost = 3 + length;

        // number_lost
        return number_lost;
    }
    // -----------------------------------------------------------------
    /*!
    Checks data structure
    (effectively const, but modifies and restores values)
    */
# ifdef NDEBUG
    void check_data_structure(void)
    {   return; }
# else
    void check_data_structure(void)
    {   // number of sets
        CPPAD_ASSERT_UNKNOWN( post_.size() == start_.size() );
        size_t n_set = start_.size();
        if( n_set == 0 )
        {   CPPAD_ASSERT_UNKNOWN( end_ == 0 );
            CPPAD_ASSERT_UNKNOWN( data_not_used_ == 0 );
            CPPAD_ASSERT_UNKNOWN( data_.size() == 0 );
            CPPAD_ASSERT_UNKNOWN( start_.size() == 0 );
            CPPAD_ASSERT_UNKNOWN( post_.size() == 0 );
            return;
        }
        // ------------------------------------------------------------------
        // save the reference counters
        pod_vector<size_t> ref_count(n_set);
        for(size_t i = 0; i < n_set; i++)
            ref_count[i] = reference_count(i);
        // ------------------------------------------------------------------

        // ------------------------------------------------------------------
        // count the number of entries in data_ that are used by sets
        size_t data_used_by_sets = 0;
        for(size_t i = 0; i < n_set; i++)
        {   size_t start = start_[i];
            if( start > 0 )
            {   // check structure for this non-empty set
                size_t reference_count = data_[start + 0];
                size_t length          = data_[start + 1];
                size_t first           = data_[start + 2];
                size_t last            = data_[start + 2 + length];
                CPPAD_ASSERT_UNKNOWN( reference_count > 0 );
                CPPAD_ASSERT_UNKNOWN( length          > 0 );
                CPPAD_ASSERT_UNKNOWN( first < end_);
                CPPAD_ASSERT_UNKNOWN( last == end_);
                //
                // decrement the reference counter
                data_[start]--;
                //
                // count the entries when find last reference
                if( data_[start] == 0 )
                {
                    // restore reference count
                    data_[start] = ref_count[i];

                    // number of data_ entries used for this set
                    data_used_by_sets += number_elements(i) + 3;
                }
            }
        }
        // ------------------------------------------------------------------
        // count the number of entries in data_ that are used by posts
        size_t data_used_by_posts = 0;
        for(size_t i = 0; i < n_set; i++)
        {   size_t post = post_[i];
            if( post > 0 )
            {   CPPAD_ASSERT_UNKNOWN( data_[post] > 0 );
                CPPAD_ASSERT_UNKNOWN( data_[post + 1] > 0 );
                CPPAD_ASSERT_UNKNOWN( data_[post + 2] < end_ );
                //
                size_t capacity     = data_[post + 1];
                data_used_by_posts += capacity + 2;
            }
        }
        // ------------------------------------------------------------------
        size_t data_used = data_used_by_sets + data_used_by_posts;
        CPPAD_ASSERT_UNKNOWN(
            data_used + data_not_used_ == data_.size()
        );
        return;
    }
# endif
    // -----------------------------------------------------------------
    /*!
    Check if one of two sets is a subset of the other set

    \param one_this
    is the index in this svec_setvec object of the first set.

    \param two_other
    is the index in other svec_setvec object of the second set.

    \param other
    is the other svec_setvec object which may be the same as this object.

    \return
    If zero, niether set is a subset of the other.
    If one, then one is a subset of two and they are not equal.
    If two, then two is a subset of one and they are not equal.
    If three, then the sets are equal.
    */
    size_t is_subset(
        size_t                     one_this    ,
        size_t                     two_other   ,
        const svec_setvec&         other       ) const
    {
        CPPAD_ASSERT_UNKNOWN( one_this  < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( two_other < other.start_.size()   );
        CPPAD_ASSERT_UNKNOWN( end_  == other.end_               );
        //
        // start
        size_t start_one    = start_[one_this];
        size_t start_two    = other.start_[two_other];
        //
        if( start_one == 0 )
        {   // set one is empty
            if( start_two == 0 )
            {   // set two is empty
                return 3;
            }
            // set one is empty and two is not empty
            return 1;
        }
        if( start_two == 0 )
        {   // set two is empty and one is not empty
            return 2;
        }
        //
        // data index
        size_t index_one    = start_one + 2;
        size_t index_two    = start_two + 2;
        //
        // value
        size_t value_one    = data_[index_one];
        size_t value_two    = other.data_[index_two];
        //
        bool one_subset     = true;
        bool two_subset     = true;
        //
        size_t value_union = std::min(value_one, value_two);
        while( (one_subset | two_subset) & (value_union < end_) )
        {   //
            if( value_one > value_union )
                two_subset = false;
            else
            {   // value_one <= value_two and value_one < end_
                value_one = data_[++index_one];
            }
            //
            if( value_two > value_union )
                one_subset = false;
            else
            {   // value_two <= value_one and value_two < end_
                value_two = other.data_[++index_two];
            }
            value_union = std::min(value_one, value_two);
        }
        if( one_subset )
        {   if( two_subset )
            {   // sets are equal
                return 3;
            }
            // one is a subset of two
            return 1;
        }
        if( two_subset )
        {   // two is a subset of one
            return 2;
        }
        //
        // neither is a subset
        return 0;
    }
    // -----------------------------------------------------------------
    /*!
    Does garbage collection when indicated.

    This routine should be called when more entries are not being used.
    If a significant propotion are not being used, the data structure
    will be compacted.

    The size of data_ should equal the number of entries used by the sets
    plus the number of entries that are not being used data_not_used_.
    Note that data_[0] never gets used.
    */
    void collect_garbage(void)
    {   if( data_not_used_ < data_.size() / 2 +  100)
            return;
        check_data_structure();
        //
        // number of sets including empty ones
        size_t n_set  = start_.size();
        //
        // use temporary to hold copy of data_ and start_
        pod_vector<size_t> data_tmp(1); // data_tmp[0] will not be used
        pod_vector<size_t> start_tmp(n_set);
        //
        for(size_t i = 0; i < n_set; i++)
        {   size_t start    = start_[i];
            if( start == 0 )
                start_tmp[i] = 0;
            else
            {   // check if this set has already been copied
                if( data_[start] == 0 )
                {   // starting address in data_tmp has been stored here
                    start_tmp[i] = data_[start + 1];
                }
                else
                {   size_t tmp_start          = data_tmp.extend(2);
                    start_tmp[i]              = tmp_start;
                    data_tmp[tmp_start + 0]   = data_[start + 0];
                    data_tmp[tmp_start + 1]   = data_[start + 1];
                    //
                    for(size_t j = 2; data_[start + j] != end_; ++j)
                        data_tmp.push_back( data_[start + j] );
                    data_tmp.push_back(end_);
                    //
                    // flag that indicates this set already copied
                    data_[start] = 0;
                    //
                    // store the starting address here
                    data_[start + 1] = tmp_start;
                }
            }
        }

        // swap the tmp and old data vectors
        start_.swap(start_tmp);
        data_.swap(data_tmp);

        // all of the elements, except the first, are used
        data_not_used_ = 1;
    }
    // -----------------------------------------------------------------
    /*!
    Assign a set equal to the union of a set and a vector;

    \param target
    is the index in this svec_setvec object of the set being assinged.

    \param left
    is the index in this svec_setvec object of the
    left operand for the union operation.
    It is OK for target and left to be the same value.

    \param right
    is a vector of size_t, sorted in accending order.
    right operand for the union operation.
    Elements can be repeated in right, but are not be repeated in the
    resulting set.
    All of the elements must have value less than end();
    */
    void binary_union(
        size_t                    target ,
        size_t                    left   ,
        const pod_vector<size_t>& right  )
    {   CPPAD_ASSERT_UNKNOWN( post_[left] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( target < start_.size() );
        CPPAD_ASSERT_UNKNOWN( left   < start_.size() );

        size_t start_left   = start_[left];
        // -------------------------------------------------------------------
        // Check if right is a subset of left so that we used reference count
        // and not make copies of identical sets.
        //
        // initialize index for left and right sets
        size_t current_left  = start_left;
        size_t current_right = 0;
        //
        // initialize value_left
        size_t value_left  = end_;
        if( current_left > 0 )
        {   // advance from reference counter to data
            current_left = current_left + 2;
            value_left   = data_[current_left];
            CPPAD_ASSERT_UNKNOWN( value_left < end_);
        }
        //
        // initialize value_right
        size_t value_right = end_;
        if( right.size() > 0 )
            value_right = right[current_right];
        //
        bool subset = true;
        while( subset & (value_right < end_) )
        {   while( value_left < value_right )
            {   // advance left
                value_left = data_[++current_left];
            }
            if( value_right < value_left )
                subset = false;
            else
            {   // advance right
                ++current_right;
                if( current_right == right.size() )
                    value_right = end_;
                else
                    value_right = right[current_right];
            }
        }
        //
        if( subset )
        {   // target = left will use reference count for identical sets
            assignment(target, left, *this);
            return;
        }

        // -------------------------------------------------------------------
        // number of elements that will be deleted by removing old version
        // of target
        size_t number_lost = drop(target);

        // drop any posting for the target set
        size_t post = post_[target];
        if( post > 0 )
        {   CPPAD_ASSERT_UNKNOWN( target != left );
            size_t capacity = data_[post + 1];
            number_lost += capacity + 2;
            post_[target] = 0;
        }
        //
        // start new version of target
        size_t start       = data_.extend(2);
        start_[target]     = start;
        data_[start]       = 1; // reference count
        // data_[start + 1] = length is not yet known
        //
        // initialize value_left
        current_left = start_left;
        value_left   = end_;
        if( current_left > 0 )
        {   // advance from reference counter to data
            current_left = current_left + 2;
            value_left   = data_[current_left];
            CPPAD_ASSERT_UNKNOWN( value_left < end_);
        }
        //
        // initialize value_right
        value_right = end_;
        if( right.size() > 0 )
            value_right = right[current_right];
        //
        // merge
        while( (value_left < end_) | (value_right < end_) )
        {   if( value_left == value_right)
            {   // advance left so left and right are no longer equal
                ++current_left;
                value_left = data_[current_left];
                CPPAD_ASSERT_UNKNOWN( value_right < value_left );
            }
            //
            if( value_left < value_right )
            {   // value_left case
                CPPAD_ASSERT_UNKNOWN( value_left < end_ );
                data_.push_back( value_left );
                //
                // advance left
                ++current_left;
                value_left = data_[current_left];
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( value_right < value_left )
                // value_right case
                CPPAD_ASSERT_UNKNOWN( value_right < end_);
                data_.push_back( value_right );
                //
                // advance right (skip values equal to this one)
                size_t previous_value = value_right;
                while( value_right == previous_value )
                {   ++current_right;
                    if( current_right == right.size() )
                        value_right = end_;
                    else
                    {   value_right = right[current_right];
                        CPPAD_ASSERT_UNKNOWN( value_right < end_ );
                    }
                }
            }
        }
        // make end of target list
        data_.push_back( end_ );
        //
        // reference count, length, and end_ are not elements of set
        CPPAD_ASSERT_UNKNOWN( data_.size() > start + 3 );
        size_t length    = data_.size() - (start + 3);
        data_[start + 1] = length;
        //

        // adjust data_not_used_
        data_not_used_ += number_lost;
        collect_garbage();
    }
public:
    /// declare a const iterator
    typedef svec_setvec_const_iterator const_iterator;
    // -----------------------------------------------------------------
    /*!
    Default constructor (no sets)
    */
    svec_setvec(void) :
    end_(0)            ,
    data_not_used_(0)  ,
    data_(0)           ,
    start_(0)          ,
    post_(0)
    { }
    // -----------------------------------------------------------------
    /// Destructor
    ~svec_setvec(void)
    {   check_data_structure();
    }
    // -----------------------------------------------------------------
    /*!
    Using copy constructor is a programing (not user) error

    \param v
    vector of sets that we are attempting to make a copy of.
    */
    svec_setvec(const svec_setvec& v)
    {   // Error: Probably a svec_setvec argument has been passed by value
        CPPAD_ASSERT_UNKNOWN(false);
    }
    // -----------------------------------------------------------------
    /*!
    Assignement operator.

    \param other
    this svec_setvec with be set to a deep copy of other.
    */
    void operator=(const svec_setvec& other)
    {   end_           = other.end_;
        data_not_used_ = other.data_not_used_;
        data_          = other.data_;
        start_         = other.start_;
        post_          = other.post_;
    }
    // -----------------------------------------------------------------
    /*!
    swap (used by move semantics version of ADFun assignment operator)

    \param other
    this sparse::list_setvec with be swapped with other.

    \par vector_of_sets
    This public member function is not yet part of
    the vector_of_sets concept.
    */
    void swap(svec_setvec& other)
    {   // size_t objects
        std::swap(end_             , other.end_);
        std::swap(data_not_used_   , other.data_not_used_);

        // pod_vectors
        data_.swap(       other.data_);
        start_.swap(      other.start_);
        post_.swap(       other.post_);
    }
    // -----------------------------------------------------------------
    /*!
    Start a new vector of sets.

    \param n_set
    is the number of sets in this vector of sets.
    \li
    If n_set is zero, any memory currently allocated for this object
    is freed.
    \li
    If n_set is non-zero, a vector of n_set sets is created and all
    the sets are initilaized as empty.

    \param end
    is the maximum element plus one (the minimum element is 0).
    If n_set is zero, end must also be zero.
    */
    void resize(size_t n_set, size_t end)
    {   check_data_structure();

        if( n_set == 0 )
        {   CPPAD_ASSERT_UNKNOWN(end == 0 );
            //
            // restore object to start after constructor
            // (no memory allocated for this object)
            data_.clear();
            start_.clear();
            post_.clear();
            data_not_used_  = 0;
            end_            = 0;
            //
            return;
        }
        end_                   = end;
        //
        start_.resize(n_set);
        post_.resize(n_set);
        for(size_t i = 0; i < n_set; i++)
        {   start_[i] = 0;
            post_[i]  = 0;
        }
        //
        data_.resize(1);     // first element is not used
        data_not_used_  = 1;
    }
    // -----------------------------------------------------------------
    /*!
    Return number of elements in a set.

    \param i
    is the index of the set we are checking number of the elements of.
    */
    size_t number_elements(size_t i) const
    {   CPPAD_ASSERT_UNKNOWN( post_[i] == 0 );
        //
        size_t start = start_[i];
        if( start == 0 )
            return 0;
        return data_[start + 1];
    }
    // ------------------------------------------------------------------
    /*!
    Post an element for delayed addition to a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the value of the element that we are posting.
    The same element may be posted multiple times.

    \par
    It is faster to post multiple elements to set i and then call
    process_post(i) then to add each element individually.
    It is an error to call any member function,
    that depends on the value of set i,
    before processing the posts to set i.
    */
    void post_element(size_t i, size_t element)
    {   CPPAD_ASSERT_UNKNOWN( i < start_.size() );
        CPPAD_ASSERT_UNKNOWN( element < end_ );

        size_t post = post_[i];
        if( post == 0 )
        {   // minimum capacity for an post vector
            size_t min_capacity = 10;
            size_t post_new = data_.extend(min_capacity + 2);
            data_[post_new]     = 1;            // length
            data_[post_new + 1] = min_capacity; // capacity
            data_[post_new + 2] = element;
            post_[i]            = post_new;
        }
        else
        {   size_t length   = data_[post];
            size_t capacity = data_[post + 1];
            if( length == capacity )
            {
                size_t post_new = data_.extend( 2 * capacity );
                //
                data_[post_new]     = length + 1;
                data_[post_new + 1] = 2 * capacity;
                //
                for(size_t j = 0; j < length; j++)
                    data_[post_new + 2 + j] = data_[post + 2 + j];
                data_[post_new + 2 + length] = element;
                //
                post_[i]            = post_new;
                size_t number_lost = length + 2;
                data_not_used_    += number_lost;
            }
            else
            {   data_[post]              = length + 1;
                data_[post + 2 + length] = element;
            }
        }

        // check amount of data_not_used_
        collect_garbage();

        return;
    }
    // -----------------------------------------------------------------
    /*!
    process post entries for a specific set.

    \param i
    index of the set for which we are processing the post entries.

    \par post_
    Upon call, post_[i] is location in data_ of the elements that get
    added to the i-th set.  Upon return, post_[i] is zero.
    */
    void process_post(size_t i)
    {   // start
        size_t start = start_[i];
        // post
        size_t post = post_[i];
        //
        // check if there are no elements to process
        if( post == 0 )
            return;
        //
        // sort the elements that need to be processed
        size_t  length_post   = data_[post];
        size_t  capacity_post = data_[post + 1];
        size_t* first_post    = data_.data() + post + 2;
        size_t* last_post     = first_post + length_post;
        std::sort(first_post, last_post);
        // -------------------------------------------------------------------
        // check if posted elements are a subset of set
        //
        // first element of the set
        size_t current_set = start;
        size_t value_set   = end_;
        if( start > 0 )
        {   current_set = start + 2;
            value_set   = data_[current_set];
        }
        //
        // first element to post
        size_t* current_post = first_post;
        size_t  value_post   = *current_post;
        //
        bool subset = true;
        while( subset & (value_post != end_) )
        {   while( value_set < value_post )
                value_set = data_[++current_set];
            if( value_post < value_set )
                subset = false;
            else
            {   ++current_post;
                if( current_post == last_post )
                    value_post = end_;
                else
                    value_post = *current_post;
            }
        }
        //
        if( subset )
        {   // drop the post_ elements
            post_[i] = 0;
            //
            size_t number_lost = capacity_post + 2;
            data_not_used_    += number_lost;
            collect_garbage();
            //
            // nothing else to do
            return;
        }
        // -------------------------------------------------------------------
        // number of element that will be lost by removing old i-th set
        size_t number_lost = drop(i);

        // start new version of i-th set
        size_t start_new  = data_.extend(2);
        start_[i]         = start_new;
        data_[start_new]  = 1; // reference count
        // data[start_new + 1] = length_new is not yet known
        //
        // first element of the set
        current_set = start;
        value_set   = end_;
        if( start > 0 )
        {   current_set = start + 2;
            value_set   = data_[current_set];
        }
        //
        // first element to process
        current_post = first_post;
        value_post   = *current_post;
        //
        // merge
        while( (value_set < end_) | (current_post != last_post ) )
        {   if( value_set == value_post )
            {   // advance left so left and right are no longer equal
                ++current_set;
                value_set = data_[current_set];
                CPPAD_ASSERT_UNKNOWN( value_post < value_set );
            }
            //
            if( value_set < value_post )
            {   // add value_set
                CPPAD_ASSERT_UNKNOWN( value_set < end_ );
                data_.push_back( value_set );
                //
                // advance set
                ++current_set;
                value_set = data_[current_set];
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( value_post < value_set )
                // add value_post
                CPPAD_ASSERT_UNKNOWN( value_post < end_);
                data_.push_back( value_post );
                //
                // advance post (skip values equal to this one)
                size_t value_previous = value_post;
                while( value_post == value_previous )
                {   ++current_post;
                    if( current_post == last_post )
                        value_post = end_;
                    else
                        value_post = *current_post;
                }
            }
        }
        // make end of target list
        data_.push_back( end_ );
        //
        // reference count, length, and end_ are not elements of set
        CPPAD_ASSERT_UNKNOWN( data_.size() > start_new + 3 );
        size_t length_new    = data_.size() - (start_new + 3);
        data_[start_new + 1] = length_new;
        CPPAD_ASSERT_UNKNOWN( data_[start_new + length_new + 2] == end_ );
        //
        // drop to post_ elements for this set
        post_[i] = 0;
        //
        number_lost    += capacity_post + 2;
        data_not_used_ += number_lost;
        collect_garbage();
        //
        return;
    }
    // -----------------------------------------------------------------
    /*!
    Add one element to a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the element we are adding to the set.
    */
    void add_element(size_t i, size_t element)
    {   CPPAD_ASSERT_UNKNOWN( i   < start_.size() );
        CPPAD_ASSERT_UNKNOWN( element < end_ );

        // check if element is already in the set
        if( is_element(i, element) )
            return;

        // check for case where old set is empty
        size_t start = start_[i];
        if( start == 0 )
        {   start            = data_.extend(4);
            start_[i]        = start;
            data_[start]     = 1;        // reference count
            data_[start + 1] = 1;        // length
            data_[start + 2] = element;  // the element
            data_[start + 3] = end_;     // end marker
            return;
        }
        //
        size_t number_lost = drop(i);
        //
        // start of new set
        size_t length         = data_[start + 1];
        size_t new_start      = data_.extend(2);
        data_[new_start]      = 1;          // reference count
        data_[new_start + 1]  = length + 1; // new length
        //
        size_t count = 0;
        size_t value     = data_[start + 2 + count];
        // before new element
        while( value < element)
        {   data_.push_back( value );
            ++count;
            value   = data_[start + 2 + count];
        }
        CPPAD_ASSERT_UNKNOWN( element < value )
        // new element
        data_.push_back( element );
        // after new element
        while( value < end_ )
        {   data_.push_back( value );
            ++count;
            value   = data_[start + 2 + count];
        }
        data_.push_back( end_ );

        //
        // connect up new set
        start_[i] = new_start;

        // adjust data_not_used_
        data_not_used_ += number_lost;
        collect_garbage();
        //
        return;
    }
    // -----------------------------------------------------------------
    /*!
    Check if an element is in a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the element we are checking to see if it is in the set.
    */
    bool is_element(size_t i, size_t element) const
    {   CPPAD_ASSERT_UNKNOWN( post_[i] == 0 );
        CPPAD_ASSERT_UNKNOWN( element < end_ );
        //
        size_t start = start_[i];
        if( start == 0 )
            return false;
        //
        size_t length       = data_[start + 1];
        const size_t* first = data_.data() + start + 2;
        const size_t* last  = first + length;
        if( length < 10 )
        {   bool result = false;
            while( last > first )
                result |= *(--last) == element;
            return result;
        }
        //
        return std::binary_search(first, last, element);
    }
    // -----------------------------------------------------------------
    /*!
    Assign the empty set to one of the sets.

    \param target
    is the index of the set we are setting to the empty set.

    \par data_not_used_
    increments this value by number of data_ elements that are lost
    (unlinked) by this operation.
    */
    void clear(size_t target)
    {
        // number of data_ elements used for this set
        size_t number_lost = drop( target );

        // set target to empty set
        start_[target] = 0;

        // drop the posted elements
        if( post_[target] != 0 )
        {   size_t capacity = post_[target + 1];
            number_lost    += capacity + 2;
            //
            post_[target] = 0;
        }

        // adjust data_not_used_
        data_not_used_ += number_lost;
        collect_garbage();
    }
    // -----------------------------------------------------------------
    /*!
    Assign one set equal to another set.

    \param this_target
    is the index in this svec_setvec object of the set being assinged.

    \param other_source
    is the index in the other svec_setvec object of the
    set that we are using as the value to assign to the target set.

    \param other
    is the other svec_setvec object (which may be the same as this
    svec_setvec object). This must have the same value for end_.

    \par data_not_used_
    increments this value by number of elements lost.
    */
    void assignment(
        size_t                  this_target  ,
        size_t                  other_source ,
        const svec_setvec&      other        )
    {   CPPAD_ASSERT_UNKNOWN( other.post_[ other_source ] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( this_target  <   start_.size()        );
        CPPAD_ASSERT_UNKNOWN( other_source <   other.start_.size()  );
        CPPAD_ASSERT_UNKNOWN( end_        == other.end_   );

        // check if we are assigning a set to itself
        if( (this == &other) & (this_target == other_source) )
            return;

        // number of elements that will be deleted by this operation
        size_t number_lost = drop(this_target);

        // drop any posting for the target set
        size_t post = post_[this_target];
        if( post > 0 )
        {   // do not need to worry about target being same as source
            size_t capacity = data_[post + 1];
            number_lost += capacity + 2;
            post_[this_target] = 0;
        }

        // If this and other are the same, use another reference to same list
        size_t other_start = other.start_[other_source];
        if( this == &other )
        {   CPPAD_ASSERT_UNKNOWN( this_target != other_source );
            start_[this_target] = other_start;
            if( other_start != 0 )
            {   // increment reference count
                data_[other_start]++;
            }
        }
        else if( other_start  == 0 )
        {   // the target list is empty
            start_[this_target] = 0;
        }
        else
        {   // make a copy of the other list in this svec_setvec
            size_t length         = other.data_[other_start + 1];
            size_t this_start     = data_.extend(2);
            start_[this_target]   = this_start;
            data_[this_start]     = 1;      // reference count
            data_[this_start + 1] = length; // length
            for(size_t j = 0; j < length; ++j)
                data_.push_back( other.data_[other_start + 2 + j] );
            data_.push_back(end_);
        }

        // adjust data_not_used_
        data_not_used_ += number_lost;
        collect_garbage();
    }
    // -----------------------------------------------------------------
    /*!
    Assign a set equal to the union of two other sets.

    \param this_target
    is the index in this svec_setvec object of the set being assinged.

    \param this_left
    is the index in this svec_setvec object of the
    left operand for the union operation.
    It is OK for this_target and this_left to be the same value.

    \param other_right
    is the index in the other svec_setvec object of the
    right operand for the union operation.
    It is OK for this_target and other_right to be the same value.

    \param other
    is the other svec_setvec object (which may be the same as this
    svec_setvec object).
    */
    void binary_union(
        size_t                this_target  ,
        size_t                this_left    ,
        size_t                other_right  ,
        const svec_setvec& other        )
    {   CPPAD_ASSERT_UNKNOWN( post_[this_left] == 0 );
        CPPAD_ASSERT_UNKNOWN( other.post_[ other_right ] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( this_target < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( this_left   < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.start_.size()   );
        CPPAD_ASSERT_UNKNOWN( end_        == other.end_           );

        // check if one of the two operands is a subset of the the other
        size_t subset = is_subset(this_left, other_right, other);

        // case where right is a subset of left or right and left are equal
        if( subset == 2 || subset == 3 )
        {   assignment(this_target, this_left, *this);
            return;
        }
        // case where the left is a subset of right and they are not equal
        if( subset == 1 )
        {   assignment(this_target, other_right, other);
            return;
        }
        // if niether case holds, then both left and right are non-empty
        CPPAD_ASSERT_UNKNOWN( reference_count(this_left) > 0 );
        CPPAD_ASSERT_UNKNOWN( other.reference_count(other_right) > 0 );

        // must get all the start indices before modify start_this
        // (in case start_this is the same as start_left or start_right)
        size_t start_left    = start_[this_left];
        size_t start_right   = other.start_[other_right];

        // number of list elements that will be deleted by this operation
        size_t number_lost = drop(this_target);

        // drop any posting for the target set
        size_t post = post_[this_target];
        if( post > 0 )
        {   // do not need to worry about target being same as left or right
            size_t capacity = data_[post + 1];
            number_lost += capacity + 2;
            post_[this_target] = 0;
        }

        // start the new list
        size_t start        = data_.extend(2);
        start_[this_target] = start;
        data_[start]        = 1; // reference count
        // data_[start + 1] = length is not yet known

        // initilaize left
        CPPAD_ASSERT_UNKNOWN( start_left != 0 );
        size_t current_left = start_left + 2;
        size_t value_left   = data_[current_left];
        CPPAD_ASSERT_UNKNOWN( value_left < end_ );

        // initilaize right
        CPPAD_ASSERT_UNKNOWN( start_right != 0 );
        size_t current_right = start_right + 2;
        size_t value_right   = other.data_[current_right];
        CPPAD_ASSERT_UNKNOWN( value_right < end_ );


        CPPAD_ASSERT_UNKNOWN( value_left < end_ && value_right < end_ );
        while( (value_left < end_) | (value_right < end_) )
        {   if( value_left == value_right )
            {   // advance right so left and right are no longer equal
                ++current_right;
                value_right = other.data_[current_right];
            }
            if( value_left < value_right )
            {   data_.push_back( value_left );
                // advance left to its next element
                ++current_left;
                value_left = data_[current_left];
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( value_right < value_left )
                data_.push_back( value_right );
                // advance right to its next element
                ++current_right;
                value_right = other.data_[current_right];
            }
        }
        // make end of target list
        data_.push_back( end_ );
        //
        // reference count, length, and end_ are not elements of set
        CPPAD_ASSERT_UNKNOWN( data_.size() > start + 3 );
        size_t length = data_.size() - (start + 3);
        data_[start + 1] = length;

        // adjust data_not_used_
        data_not_used_ += number_lost;
        collect_garbage();
    }
    // -----------------------------------------------------------------
    /*!
    Assign a set equal to the intersection of two other sets.

    \param this_target
    is the index in this svec_setvec object of the set being assinged.

    \param this_left
    is the index in this svec_setvec object of the
    left operand for the intersection operation.
    It is OK for this_target and this_left to be the same value.

    \param other_right
    is the index in the other svec_setvec object of the
    right operand for the intersection operation.
    It is OK for this_target and other_right to be the same value.

    \param other
    is the other svec_setvec object (which may be the same as this
    svec_setvec object).
    */
    void binary_intersection(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const svec_setvec&      other        )
    {   CPPAD_ASSERT_UNKNOWN( post_[this_left] == 0 );
        CPPAD_ASSERT_UNKNOWN( other.post_[ other_right ] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( this_target < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( this_left   < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.start_.size()   );
        CPPAD_ASSERT_UNKNOWN( end_        == other.end_           );
        //
        // check if one of the two operands is a subset of the the other
        size_t subset = is_subset(this_left, other_right, other);

        // case where left is a subset of right or left and right are equal
        if( subset == 1 || subset == 3 )
        {   assignment(this_target, this_left, *this);
            return;
        }
        // case where the right is a subset of left and they are not equal
        if( subset == 2 )
        {   assignment(this_target, other_right, other);
            return;
        }
        // if niether case holds, then both left and right are non-empty
        CPPAD_ASSERT_UNKNOWN( reference_count(this_left) > 0 );
        CPPAD_ASSERT_UNKNOWN( other.reference_count(other_right) > 0 );

        // must get all the start indices before modify start_this
        // (in case start_this is the same as start_left or start_right)
        size_t start_left    = start_[this_left];
        size_t start_right   = other.start_[other_right];


        // number of list elements that will be deleted by this operation
        size_t number_lost = drop(this_target);

        // drop any posting for the target set
        size_t post = post_[this_target];
        if( post > 0 )
        {   // do not need to worry about target being same as left or right
            size_t capacity = data_[post + 1];
            number_lost += capacity + 2;
            post_[this_target] = 0;
        }

        // initialize intersection as empty
        size_t start        = 0;
        start_[this_target] = start;

        // initilaize left
        CPPAD_ASSERT_UNKNOWN( start_left != 0 );
        size_t current_left = start_left + 2;
        size_t value_left   = data_[current_left];
        CPPAD_ASSERT_UNKNOWN( value_left < end_ );

        // initilaize right
        CPPAD_ASSERT_UNKNOWN( start_right != 0 );
        size_t current_right = start_right + 2;
        size_t value_right   = other.data_[current_right];
        CPPAD_ASSERT_UNKNOWN( value_right < end_ );

        while( (value_left < end_) & (value_right < end_) )
        {   if( value_left == value_right )
            {   if( start == 0 )
                {   // this is the first element in the intersection
                    start               = data_.extend(2);
                    start_[this_target] = start;
                    data_[start]        = 1; // reference count
                    // data_[start + 1] = length is not yet known
                }
                data_.push_back( value_left );
                //
                // advance left to its next element
                ++current_left;
                value_left = data_[current_left];
            }
            if( value_left > value_right )
            {   // advance right
                ++current_right;
                value_right = other.data_[current_right];
            }
            if( value_right > value_left )
            {   // advance left
                ++current_left;
                value_left = data_[current_left];
            }
        }
        if( start != 0 )
        {   data_.push_back(end_);
            CPPAD_ASSERT_UNKNOWN( data_.size() > start + 3 );
            size_t length = data_.size() - (start + 3);
            data_[start + 1] = length;
        }

        // adjust data_not_used_
        data_not_used_ += number_lost;
        collect_garbage();
    }
    // -----------------------------------------------------------------
    /*! Fetch n_set for vector of sets object.

    \return
    Number of from sets for this vector of sets object
    */
    size_t n_set(void) const
    {   return start_.size(); }
    // -----------------------------------------------------------------
    /*! Fetch end for this vector of sets object.

    \return
    is the maximum element value plus one (the minimum element value is 0).
    */
    size_t end(void) const
    {   return end_; }
    // -----------------------------------------------------------------
    /*! Amount of memory used by this vector of sets

    \return
    The amount of memory in units of type unsigned char memory.
    */
    size_t memory(void) const
    {   return data_.capacity() * sizeof(size_t);
    }
    /*!
    Print the vector of sets (used for debugging)
    */
    void print(void) const;
};
// =========================================================================
/*!
cons_iterator for one set of positive integers in a svec_setvec object.

All the public member functions for this class are also in the
sparse::pack_setvec_const_iterator and sparse::list_setvec_const_iterator classes.
This defines the CppAD vector_of_sets iterator concept.
*/
class svec_setvec_const_iterator {
private:
    /// data for the entire vector of sets
    const pod_vector<size_t>& data_;

    /// Possible elements in a list are 0, 1, ..., end_ - 1;
    const size_t                   end_;

    /// data index of next entry, zero for no more entries
    size_t                         data_index_;
public:
    /// construct a const_iterator for a set in a svec_setvec object
    svec_setvec_const_iterator (const svec_setvec& vec_set, size_t i)
    :
    data_( vec_set.data_ ) ,
    end_ ( vec_set.end_ )
    {   CPPAD_ASSERT_UNKNOWN( vec_set.post_[i] == 0 );
        //
        size_t start = vec_set.start_[i];
        if( start == 0 )
        {   data_index_ = 0;
        }
        else
        {   // data index of the first element in the set
            data_index_ = start + 2;
            CPPAD_ASSERT_UNKNOWN( data_[data_index_] < end_ );
        }
    }

    /// advance to next element in this list
    svec_setvec_const_iterator& operator++(void)
    {   if( data_index_ != 0 )
        {   ++data_index_;
            if( data_[data_index_] == end_ )
                data_index_ = 0;
        }
        return *this;
    }

    /// obtain value of this element of the set of positive integers
    /// (end_ for no such element)
    size_t operator*(void)
    {   if( data_index_ == 0 )
            return end_;
        return data_[data_index_];
    }
};
// =========================================================================
/*!
Print the vector of sets (used for debugging)
*/
inline void svec_setvec::print(void) const
{   std::cout << "svec_setvec:\n";
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

/*!
Copy a user vector of sets sparsity pattern to an internal svec_setvec object.

\tparam SetVector
is a simple vector with elements of type std::set<size_t>.

\param internal
The input value of sparisty does not matter.
Upon return it contains the same sparsity pattern as user
(or the transposed sparsity pattern).

\param user
sparsity pattern that we are placing internal.

\param n_set
number of sets (rows) in the internal sparsity pattern.

\param end
end of set value (number of columns) in the interanl sparsity pattern.

\param transpose
if true, the user sparsity patter is the transposed.

\param error_msg
is the error message to display if some values in the user sparstiy
pattern are not valid.
*/
template<class SetVector>
void sparsity_user2internal(
    svec_setvec&            internal  ,
    const SetVector&        user      ,
    size_t                  n_set     ,
    size_t                  end       ,
    bool                    transpose ,
    const char*             error_msg )
{
# ifndef NDEBUG
    if( transpose )
        CPPAD_ASSERT_KNOWN( end == size_t( user.size() ), error_msg);
    if( ! transpose )
        CPPAD_ASSERT_KNOWN( n_set == size_t( user.size() ), error_msg);
# endif

    // iterator for user set
    std::set<size_t>::const_iterator itr;

    // size of internal sparsity pattern
    internal.resize(n_set, end);

    if( transpose )
    {   // transposed pattern case
        for(size_t j = 0; j < end; j++)
        {   itr = user[j].begin();
            while(itr != user[j].end())
            {   size_t i = *itr++;
                CPPAD_ASSERT_KNOWN(i < n_set, error_msg);
                internal.add_element(i, j);
            }
        }
    }
    else
    {   for(size_t i = 0; i < n_set; i++)
        {   itr = user[i].begin();
            while(itr != user[i].end())
            {   size_t j = *itr++;
                CPPAD_ASSERT_KNOWN( j < end, error_msg);
                internal.add_element(i, j);
            }
        }
    }
    return;
}

} } } // END_CPPAD_LOCAL_SPARSE_NAMESPACE
# endif
