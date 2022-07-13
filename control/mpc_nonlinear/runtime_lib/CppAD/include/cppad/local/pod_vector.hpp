# ifndef CPPAD_LOCAL_POD_VECTOR_HPP
# define CPPAD_LOCAL_POD_VECTOR_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# if CPPAD_CSTDINT_HAS_8_TO_64
# include <cstdint>
# endif
# include <cstring>
# include <algorithm>
# include <cppad/utility/thread_alloc.hpp>
# include <cppad/core/cppad_assert.hpp>
# include <cppad/local/is_pod.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file pod_vector.hpp
File used to define pod_vector classes
*/
// ---------------------------------------------------------------------------
/*!
A vector class with that does not use element constructors or destructors
(elements are Plain Old Data; i.e., is_pod<Type> must be true).

*/
template <class Type>
class pod_vector {
private:
    /// maximum number of bytes current allocation can hold
    size_t byte_capacity_;

    /// number of bytes currently in this vector
    size_t byte_length_;

    /// pointer to the first type elements
    /// (not defined and should not be used when byte_capacity_ = 0)
    Type   *data_;

    /// do not use the copy constructor
    explicit pod_vector(const pod_vector& )
    {   CPPAD_ASSERT_UNKNOWN(false); }
public:
    /// default constructor sets byte_capacity_ = byte_length_ = data_ = 0
    pod_vector(void)
    : byte_capacity_(0), byte_length_(0), data_(nullptr)
    {   CPPAD_ASSERT_UNKNOWN( is_pod<Type>() );
    }

    /// sizing constructor
    pod_vector(
        /// number of elements in this vector
        size_t n )
    : byte_capacity_(0), byte_length_(0), data_(nullptr)
    {   CPPAD_ASSERT_UNKNOWN( is_pod<Type>() );
        extend(n);
    }

    /// Destructor: returns allocated memory to thread_alloc;
    /// see extend and resize.  If this is not plain old data,
    /// the destructor for each element is called.
    ~pod_vector(void)
    {   if( byte_capacity_ > 0 )
        {
            void* v_ptr = reinterpret_cast<void*>( data_ );
            thread_alloc::return_memory(v_ptr);
        }
    }

    /*
    Return a pointer to a pod_vector with a different type of element.

    - This vector and the other share the same memory.

    - The the other vector should not be deleted.

    - The following operations work the same for this and the other vector:
    swap, clear, assignment.
    */
    template <class Other>
    pod_vector<Other>* pod_vector_ptr(void)
    {   return reinterpret_cast< pod_vector<Other>* >(this);
    }
    template <class Other>
    const pod_vector<Other>* pod_vector_ptr(void) const
    {   return reinterpret_cast< const pod_vector<Other>* >(this);
    }

    /// current number of elements in this vector.
    size_t size(void) const
    {   return byte_length_ / sizeof(Type); }

    /// current capacity (amount of allocated storage) for this vector.
    size_t capacity(void) const
    {   return byte_capacity_ / sizeof(Type); }

    /// current data pointer is no longer valid after any of the following:
    /// extend, resize, erase, clear, assignment and destructor.
    Type* data(void)
    {   return data_; }

    /// const version of data pointer (see non-const documentation)
    const Type* data(void) const
    {   return data_; }

    //  ----------------------------------------------------------------------
    /// non-constant element access; i.e., we can change this element value
    Type& operator[](
        /// element index, must be less than length
        size_t i
    )
    {   CPPAD_ASSERT_UNKNOWN( i * sizeof(Type) < byte_length_ );
        return data_[i];
    }
    /// non-constant element access; i.e., we can change this element value
    template <class Index>
    Type& operator[](
        /// element index, must be less than length and convertable to size_t
        Index i
    )
    {   return (*this)[size_t(i)]; }
    //  ----------------------------------------------------------------------
    /// constant element access; i.e., we cannot change this element value
    const Type& operator[](
        /// element index, must be less than length
        size_t i
    ) const
    {   CPPAD_ASSERT_UNKNOWN( i * sizeof(Type) < byte_length_ );
        return data_[i];
    }
    /// constant element access; i.e., we cannot change this element value
    template <class Index>
    const Type& operator[](
        /// element index, must be less than length and convertable to size_t
        Index i
    ) const
    {   return (*this)[size_t(i)]; }
    //  ----------------------------------------------------------------------

    /*!
    Add an element to theh back of this vector

    \param e
    is the element we are adding to the back of the vector.
    */
    void push_back(const Type& e)
    {   size_t i = extend(1);
        data_[i] = e;
    }

    /*!
    Swap all properties of this vector with another.
    This is useful when moving a vector that grows after it has reached
    its final size (without copying every element).

    \param other
    is the other vector that we are swapping this vector with.
    */
    void swap(pod_vector& other)
    {   std::swap(byte_capacity_, other.byte_capacity_);
        std::swap(byte_length_,   other.byte_length_);
        std::swap(data_,          other.data_);
    }
    // ----------------------------------------------------------------------
    /*!
    Increase the number of elements the end of this vector
    (existing elements are always preserved).

    \param n
    is the number of elements to add to end of this vector.

    \return
    is the number of elements in the vector before it was extended.
    This is the index of the first new element added to the vector.

    - If Type is plain old data, new elements are not initialized;
    i.e., their constructor is not called. Otherwise, the constructor
    is called for each new element.

    - This and resize are the only routine that allocate memory for
    pod_vector. They uses thread_alloc for this allocation.
    */
    size_t extend(size_t n)
    {   size_t old_length   = byte_length_;
        byte_length_       += n * sizeof(Type);

        // check if we can use current memory
        if( byte_length_ <= byte_capacity_ )
            return old_length / sizeof(Type);

        // save more old information
        size_t old_capacity = byte_capacity_;
        void* old_v_ptr     = reinterpret_cast<void*>(data_);

        // get new memory and set capacity
        void* v_ptr = thread_alloc::get_memory(byte_length_, byte_capacity_);
        data_       = reinterpret_cast<Type*>(v_ptr);

        // copy old data to new
        if( old_length >  0 )
            std::memcpy(v_ptr, old_v_ptr, old_length);

        // return old memory to available pool
        if( old_capacity > 0 )
            thread_alloc::return_memory(old_v_ptr);

        // return value for extend(n) is the old length
        CPPAD_ASSERT_UNKNOWN( byte_length_ <= byte_capacity_ );
        return old_length / sizeof(Type);
    }
    // ----------------------------------------------------------------------
    /*!
    resize the vector (existing elements preserved when n <= capacity() ).

    \param n
    is the new size for this vector.

    \par
    if n <= capacity(), no memory is freed or allocated, the capacity
    is not changed, and existing elements are preserved.
    If n > capacity(), new memory is allocates and all the
    data in the vector is lost.

    - If  Type is plain old data, new elements are not initialized;
    i.e., their constructor is not called. Otherwise, the constructor
    is called for each new element.

    - This and extend are the only routine that allocate memory for
    pod_vector. They uses thread_alloc for this allocation.
    */
    void resize(size_t n)
    {   byte_length_ = n * sizeof(Type);

        // check if we must allocate new memory
        if( byte_capacity_ < byte_length_ )
        {   void* v_ptr;
            //
            if( byte_capacity_ > 0 )
            {   // return old memory to available pool
                v_ptr = reinterpret_cast<void*>( data_ );
                thread_alloc::return_memory(v_ptr);
            }
            //
            // get new memory and set capacity
            v_ptr     = thread_alloc::get_memory(byte_length_, byte_capacity_);
            data_     = reinterpret_cast<Type*>(v_ptr);
            //
        }
        CPPAD_ASSERT_UNKNOWN( byte_length_ <= byte_capacity_ );
    }
    // ----------------------------------------------------------------------
    /*!
    Remove all the elements from this vector and free its memory.
    */
    void clear(void)
    {   if( byte_capacity_ > 0 )
        {
            void* v_ptr = reinterpret_cast<void*>( data_ );
            thread_alloc::return_memory(v_ptr);
        }
        data_          = nullptr;
        byte_capacity_ = 0;
        byte_length_   = 0;
    }
    // -----------------------------------------------------------------------
    /// vector assignment operator
    void operator=(
        /// right hand size of the assingment operation
        const pod_vector& x
    )
    {   CPPAD_ASSERT_UNKNOWN( x.byte_length_ % sizeof(Type) == 0 );
        resize( x.byte_length_ / sizeof(Type) );
        if( byte_length_ > 0 )
        {
            void* v_ptr   = reinterpret_cast<void*>( data_ );
            void* v_ptr_x = reinterpret_cast<void*>( x.data_ );
            std::memcpy(v_ptr, v_ptr_x, byte_length_);
        }

    }
};
// ---------------------------------------------------------------------------
/*!
A vector class with that does not use element constructors or destructors
when is_pod<Type> is true.
*/
template <class Type>
class pod_vector_maybe {
private:
    /// maximum number of Type elements current allocation can hold
    size_t capacity_;

    /// number of elements currently in this vector
    size_t length_;

    /// pointer to the first type elements
    /// (not defined and should not be used when capacity_ = 0)
    Type   *data_;

    /// do not use the copy constructor
    explicit pod_vector_maybe(const pod_vector_maybe& )
    {   CPPAD_ASSERT_UNKNOWN(false); }
public:
    /// default constructor sets capacity_ = length_ = data_ = 0
    pod_vector_maybe(void)
    : capacity_(0), length_(0), data_(nullptr)
    {   CPPAD_ASSERT_UNKNOWN( is_pod<size_t>() );
    }

    /// sizing constructor
    pod_vector_maybe(
        /// number of elements in this vector
        size_t n )
    : capacity_(0), length_(0), data_(nullptr)
    {   extend(n); }


    /// Destructor: returns allocated memory to thread_alloc;
    /// see extend and resize.  If this is not plain old data,
    /// the destructor for each element is called.
    ~pod_vector_maybe(void)
    {   if( capacity_ > 0 )
        {   if( ! is_pod<Type>() )
            {   // call destructor for each element
                for(size_t i = 0; i < capacity_; i++)
                    (data_ + i)->~Type();
            }
            void* v_ptr = reinterpret_cast<void*>( data_ );
            thread_alloc::return_memory(v_ptr);
        }
    }

    /// current number of elements in this vector.
    size_t size(void) const
    {   return length_; }

    /// current capacity (amount of allocated storage) for this vector.
    size_t capacity(void) const
    {   return capacity_; }

    /// current data pointer is no longer valid after any of the following:
    /// extend, resize, erase, clear, assignment, and destructor.
    Type* data(void)
    {   return data_; }

    /// const version of data pointer (see non-const documentation)
    const Type* data(void) const
    {   return data_; }
    // ----------------------------------------------------------------------
    /// non-constant element access; i.e., we can change this element value
    Type& operator[](
        /// element index, must be less than length
        size_t i
    )
    {   CPPAD_ASSERT_UNKNOWN( i < length_ );
        return data_[i];
    }
    /// non-constant element access; i.e., we can change this element value
    template <class Index>
    Type& operator[](
        /// element index, must be less than length and convertable to size_t
        Index i
    )
    {   return (*this)[size_t(i)]; }

    // ----------------------------------------------------------------------
    /// constant element access; i.e., we cannot change this element value
    const Type& operator[](
        /// element index, must be less than length
        size_t i
    ) const
    {   CPPAD_ASSERT_UNKNOWN( i < length_ );
        return data_[i];
    }
    /// constant element access; i.e., we cannot change this element value
    template <class Index>
    const Type& operator[](
        /// element index, must be less than length and convertable to size_t
        Index i
    ) const
    {   return (*this)[size_t(i)]; }

    // ----------------------------------------------------------------------
    /*!
    Add an element to theh back of this vector

    \param e
    is the element we are adding to the back of the vector.
    */
    void push_back(const Type& e)
    {   size_t i = extend(1);
        data_[i] = e;
    }

    /*!
    Swap all properties of this vector with another.
    This is useful when moving a vector that grows after it has reached
    its final size (without copying every element).

    \param other
    is the other vector that we are swapping this vector with.
    */
    void swap(pod_vector_maybe& other)
    {   std::swap(capacity_, other.capacity_);
        std::swap(length_,   other.length_);
        std::swap(data_,     other.data_);
    }
    // ----------------------------------------------------------------------
    /*!
    Increase the number of elements the end of this vector
    (existing elements are always preserved).

    \param n
    is the number of elements to add to end of this vector.

    \return
    is the number of elements in the vector before it was extended.
    This is the index of the first new element added to the vector.

    - If Type is plain old data, new elements are not initialized;
    i.e., their constructor is not called. Otherwise, the constructor
    is called for each new element.

    - This and resize are the only routine that allocate memory for
    pod_vector_maybe. They uses thread_alloc for this allocation.
    */
    size_t extend(size_t n)
    {   size_t old_length   = length_;
        length_            += n;

        // check if we can use current memory
        if( length_ <= capacity_ )
            return old_length;

        // save more old information
        size_t old_capacity = capacity_;
        Type* old_data      = data_;

        // get new memory and set capacity
        size_t length_bytes = length_ * sizeof(Type);
        size_t capacity_bytes;
        void* v_ptr = thread_alloc::get_memory(length_bytes, capacity_bytes);
        capacity_   = capacity_bytes / sizeof(Type);
        data_       = reinterpret_cast<Type*>(v_ptr);

        if( ! is_pod<Type>() )
        {   // call constructor for each new element
            for(size_t i = 0; i < capacity_; i++)
                new(data_ + i) Type();
        }

        // copy old data to new
        for(size_t i = 0; i < old_length; i++)
            data_[i] = old_data[i];

        // return old memory to available pool
        if( old_capacity > 0 )
        {   if( ! is_pod<Type>() )
            {   for(size_t i = 0; i < old_capacity; i++)
                    (old_data + i)->~Type();
            }
            v_ptr = reinterpret_cast<void*>( old_data );
            thread_alloc::return_memory(v_ptr);
        }

        // return value for extend(n) is the old length
        CPPAD_ASSERT_UNKNOWN( length_ <= capacity_ );
        return old_length;
    }
    // ----------------------------------------------------------------------
    /*!
    resize the vector (existing elements preserved when n <= capacity_).

    \param n
    is the new size for this vector.

    \par
    if n <= capacity(), no memory is freed or allocated, the capacity
    is not changed, and existing elements are preserved.
    If n > capacity(), new memory is allocates and all the
    data in the vector is lost.

    - If  Type is plain old data, new elements are not initialized;
    i.e., their constructor is not called. Otherwise, the constructor
    is called for each new element.

    - This and extend are the only routine that allocate memory for
    pod_vector_maybe. They uses thread_alloc for this allocation.
    */
    void resize(size_t n)
    {   length_ = n;

        // check if we must allocate new memory
        if( capacity_ < length_ )
        {   void* v_ptr;
            //
            // return old memory to available pool
            if( capacity_ > 0 )
            {   if( ! is_pod<Type>() )
                {   // call destructor for each old element
                    for(size_t i = 0; i < capacity_; i++)
                        (data_ + i)->~Type();
                }
                v_ptr = reinterpret_cast<void*>( data_ );
                thread_alloc::return_memory(v_ptr);
            }
            //
            // get new memory and set capacity
            size_t length_bytes = length_ * sizeof(Type);
            size_t capacity_bytes;
            v_ptr     = thread_alloc::get_memory(length_bytes, capacity_bytes);
            capacity_ = capacity_bytes / sizeof(Type);
            data_     = reinterpret_cast<Type*>(v_ptr);
            //
            CPPAD_ASSERT_UNKNOWN( length_ <= capacity_ );
            //
            if( ! is_pod<Type>() )
            {   // call constructor for each new element
                for(size_t i = 0; i < capacity_; i++)
                    new(data_ + i) Type();
            }
        }
    }
    // ----------------------------------------------------------------------
    /*!
    Remove all the elements from this vector and free its memory.
    */
    void clear(void)
    {   if( capacity_ > 0 )
        {   if( ! is_pod<Type>() )
            {   // call destructor for each element
                for(size_t i = 0; i < capacity_; i++)
                    (data_ + i)->~Type();
            }
            void* v_ptr = reinterpret_cast<void*>( data_ );
            thread_alloc::return_memory(v_ptr);
        }
        data_     = nullptr;
        capacity_ = 0;
        length_   = 0;
    }
    // -----------------------------------------------------------------------
    /// vector assignment operator
    void operator=(
        /// right hand size of the assingment operation
        const pod_vector_maybe& x
    )
    {   resize( x.length_ );
        //
        CPPAD_ASSERT_UNKNOWN( length_   == x.length_ );
        for(size_t i = 0; i < length_; i++)
        {   data_[i] = x.data_[i]; }
    }
};

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
