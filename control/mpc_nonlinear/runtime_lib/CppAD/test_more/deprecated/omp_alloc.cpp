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
$begin omp_alloc.cpp$$
$spell
    openmp
$$

$section OpenMP Memory Allocator: Example and Test$$


$head Deprecated 2011-08-31$$
This example is only intended to help convert calls to $cref omp_alloc$$
to calls to $cref thread_alloc$$.

$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/utility/omp_alloc.hpp>
# include <cppad/utility/memory_leak.hpp>
# include <vector>

namespace { // Begin empty namespace

bool omp_alloc_bytes(void)
{   bool ok = true;
    using CppAD::omp_alloc;
    size_t thread;

    // check initial memory values
    ok &= ! CppAD::memory_leak();

    // amount of static memory used by thread zero
    size_t static_inuse = omp_alloc::inuse(0);

    // determine the currently executing thread
    // (should be zero because not in parallel mode)
    thread = omp_alloc::get_thread_num();

    // repeatedly allocate enough memory for at least two size_t values.
    size_t min_size_t = 2;
    size_t min_bytes  = min_size_t * sizeof(size_t);
    size_t n_outter   = 10;
    size_t n_inner    = 5;
    size_t cap_bytes(0), i, j, k;
    for(i = 0; i < n_outter; i++)
    {   // Do not use CppAD::vector here because its use of omp_alloc
        // complicates the inuse and avaialble results.
        std::vector<void*> v_ptr(n_inner);
        for( j = 0; j < n_inner; j++)
        {   // allocate enough memory for min_size_t size_t objects
            v_ptr[j]    = omp_alloc::get_memory(min_bytes, cap_bytes);
            size_t* ptr = reinterpret_cast<size_t*>(v_ptr[j]);
            // determine the number of size_t values we have obtained
            size_t  cap_size_t = cap_bytes / sizeof(size_t);
            ok                &= min_size_t <= cap_size_t;
            // use placement new to call the size_t copy constructor
            for(k = 0; k < cap_size_t; k++)
                new(ptr + k) size_t(i + j + k);
            // check that the constructor worked
            for(k = 0; k < cap_size_t; k++)
                ok &= ptr[k] == (i + j + k);
        }
        // check that n_inner * cap_bytes are inuse and none are available
        ok &= omp_alloc::inuse(thread) == n_inner*cap_bytes + static_inuse;
        ok &= omp_alloc::available(thread) == 0;
        // return the memrory to omp_alloc
        for(j = 0; j < n_inner; j++)
            omp_alloc::return_memory(v_ptr[j]);
        // check that now n_inner * cap_bytes are now available
        // and none are in use
        ok &= omp_alloc::inuse(thread) == static_inuse;
        ok &= omp_alloc::available(thread) == n_inner * cap_bytes;
    }
    // return all the available memory to the system
    omp_alloc::free_available(thread);
    ok &= ! CppAD::memory_leak();

    return ok;
}

class my_char {
public:
    char ch_ ;
    my_char(void) : ch_(' ')
    { }
    my_char(const my_char& my_ch) : ch_(my_ch.ch_)
    { }
};

bool omp_alloc_array(void)
{   bool ok = true;
    using CppAD::omp_alloc;
    size_t i;

    // check initial memory values
    size_t thread = omp_alloc::get_thread_num();
    ok &= thread == 0;
    ok &= ! CppAD::memory_leak();
    size_t static_inuse = omp_alloc::inuse(0);

    // initial allocation of an array
    size_t  size_min  = 3;
    size_t  size_one;
    my_char *array_one  =
        omp_alloc::create_array<my_char>(size_min, size_one);

    // check the values and change them to null 'x'
    for(i = 0; i < size_one; i++)
    {   ok &= array_one[i].ch_ == ' ';
        array_one[i].ch_ = 'x';
    }

    // now create a longer array
    size_t size_two;
    my_char *array_two =
        omp_alloc::create_array<my_char>(2 * size_min, size_two);

    // check the values in array one
    for(i = 0; i < size_one; i++)
        ok &= array_one[i].ch_ == 'x';

    // check the values in array two
    for(i = 0; i < size_two; i++)
        ok &= array_two[i].ch_ == ' ';

    // check the amount of inuse and available memory
    // (an extra size_t value is used for each memory block).
    size_t check = static_inuse + sizeof(my_char)*(size_one + size_two);
    ok   &= omp_alloc::inuse(thread) - check < sizeof(my_char);
    ok   &= omp_alloc::available(thread) == 0;

    // delete the arrays
    omp_alloc::delete_array(array_one);
    omp_alloc::delete_array(array_two);
    ok   &= omp_alloc::inuse(thread) == static_inuse;
    check = sizeof(my_char)*(size_one + size_two);
    ok   &= omp_alloc::available(thread) - check < sizeof(my_char);

    // free the memory for use by this thread
    omp_alloc::free_available(thread);
    ok &= ! CppAD::memory_leak();

    return ok;
}
} // End empty namespace

bool omp_alloc(void)
{   bool ok  = true;
    using CppAD::omp_alloc;

    // check initial state of allocator
    ok  &= omp_alloc::get_max_num_threads() == 1;

    // set the maximum number of threads greater than one
    // so that omp_alloc holds onto memory
    CppAD::omp_alloc::set_max_num_threads(2);
    ok  &= omp_alloc::get_max_num_threads() == 2;
    ok  &= ! CppAD::memory_leak();

    // now use memory allocator in state where it holds onto memory
    ok   &= omp_alloc_bytes();
    ok   &= omp_alloc_array();

    // check that the tests have not held onto memory
    ok  &= ! CppAD::memory_leak();

    // set the maximum number of threads back to one
    // so that omp_alloc no longer holds onto memory
    CppAD::omp_alloc::set_max_num_threads(1);

    return ok;
}


// END C++
