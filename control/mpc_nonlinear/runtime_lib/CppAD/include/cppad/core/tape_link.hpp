# ifndef CPPAD_CORE_TAPE_LINK_HPP
# define CPPAD_CORE_TAPE_LINK_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/local/define.hpp>
# include <cppad/utility/thread_alloc.hpp>
# include <cppad/core/cppad_assert.hpp>

// needed before one can use CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL
# include <cppad/utility/thread_alloc.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE
/*!
\file tape_link.hpp
Routines that Link AD<Base> and local::ADTape<Base> Objects.

The routines that connect the AD<Base> class to the corresponding tapes
(one for each thread).
*/

/*!
Pointer to the tape identifier for this AD<Base> class and the specific thread.

\tparam Base
is the base type for this AD<Base> class.

\param thread
is the thread number. The following condition must hold
\code
(! thread_alloc::in_parallel()) || thread == thread_alloc::thread_num()
\endcode

\return
is a pointer to the tape identifier for this thread and AD<Base> class.
*/
template <class Base>
tape_id_t* AD<Base>::tape_id_ptr(size_t thread)
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;
    static tape_id_t tape_id_table[CPPAD_MAX_NUM_THREADS];
    CPPAD_ASSERT_UNKNOWN(
        (! thread_alloc::in_parallel()) || thread == thread_alloc::thread_num()
    );
    return tape_id_table + thread;
}

/*!
Handle for the tape for this AD<Base> class and the specific thread.

\tparam Base
is the base type for this AD<Base> class.


\param thread
is the thread number; i.e.,
\code
(! thread_alloc::in_parallel()) || thread == thread_alloc::thread_num()
\endcode

\return
is a handle for the tape for this AD<Base> class and the specified thread.
*/
template <class Base>
local::ADTape<Base>** AD<Base>::tape_handle(size_t thread)
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;
    static local::ADTape<Base>* tape_table[CPPAD_MAX_NUM_THREADS];
    CPPAD_ASSERT_UNKNOWN(
        (! thread_alloc::in_parallel()) || thread == thread_alloc::thread_num()
    );
    return tape_table + thread;
}

/*!
Pointer for the tape for this AD<Base> class and the current thread.

\code
thread == thread_alloc::thread_num()
\endcode

\tparam Base
is the base type corresponding to AD<Base> operations.

\return
is a pointer to the tape that is currently recording AD<Base> operations
for the current thread.
If this value is nullptr, there is no tape currently
recording AD<Base> operations for this thread.
*/
template <class Base>
local::ADTape<Base>* AD<Base>::tape_ptr(void)
{   size_t thread = thread_alloc::thread_num();
    return *tape_handle(thread);
}

/*!
Pointer for the tape for this AD<Base> class and the specified tape
identifier.

\tparam Base
is the base type corresponding to AD<Base> operations.

\param tape_id
is the identifier for the tape that is currently recording
AD<Base> operations for the current thread.
It must hold that the current thread is
\code
    thread = size_t( tape_id % CPPAD_MAX_NUM_THREADS )
\endcode
and that there is a tape recording AD<Base> operations
for this thread.
If this is not the currently executing thread,
a variable from a different thread is being recorded on the
tape for this thread which is a user error.

\return
is a pointer to the tape that is currently recording AD<Base> operations
for the current thread (and it is not nullptr).

\par Restrictions
This routine should only be called if there is a tape recording operaitons
for the specified thread.
*/
template <class Base>
local::ADTape<Base>* AD<Base>::tape_ptr(tape_id_t tape_id)
{   size_t thread = size_t( tape_id % CPPAD_MAX_NUM_THREADS );
    CPPAD_ASSERT_KNOWN(
        thread == thread_alloc::thread_num(),
        "Attempt to use an AD variable with two different threads."
    );
    CPPAD_ASSERT_UNKNOWN( tape_id == *tape_id_ptr(thread) );
    CPPAD_ASSERT_UNKNOWN( *tape_handle(thread) != nullptr );
    return *tape_handle(thread);
}

/*!
Create and delete tapes that record AD<Base> operations for current thread.

\par thread
the current thread is given by
\code
thread = thread_alloc::thread_num()
\endcode

\tparam Base
is the base type corresponding to AD<Base> operations.

\param job
This argument determines if we are creating a new tape, or deleting an
old one.

- new_tape_manage :
Creates and a new tape.
It is assumed that there is no tape recording AD<Base> operations
for this thread when tape_manage is called.

- delete_tape_manage :
It is assumed that there is a tape recording AD<Base> operations
for this thread when tape_manage is called.
The value of <tt>*tape_id_ptr(thread)</tt> will be advanced by
 CPPAD_MAX_NUM_THREADS.


\return
- <tt>job == new_tape_manage</tt>: a pointer to the new tape is returned.
- <tt>job == delete_tape_manage</tt>: the value nullptr is returned.
*/
template <class Base>
local::ADTape<Base>*  AD<Base>::tape_manage(tape_manage_enum job)
{
    CPPAD_ASSERT_UNKNOWN(
        job == new_tape_manage || job == delete_tape_manage
    );
    // thread, tape_id, and tape for this call
    size_t                thread     = thread_alloc::thread_num();
    tape_id_t*            tape_id_p  = tape_id_ptr(thread);
    local::ADTape<Base>** tape_h     = tape_handle(thread);


    // -----------------------------------------------------------------------
    // new_tape_manage
    if( job == new_tape_manage )
    {
        // tape for this thread must be null at the start
        CPPAD_ASSERT_UNKNOWN( *tape_h  == nullptr );

        // allocate separate memroy to avoid false sharing
        *tape_h = new local::ADTape<Base>();

        // if tape id is zero, initialize it so that
        // thread == tape id % CPPAD_MAX_NUM_THREADS
        if( *tape_id_p == 0 )
        {   size_t new_tape_id = thread + CPPAD_MAX_NUM_THREADS;
            CPPAD_ASSERT_KNOWN(
                size_t( std::numeric_limits<tape_id_t>::max() ) >= new_tape_id,
                "cppad_tape_id_type maximum value has been exceeded"
            );
            *tape_id_p = static_cast<tape_id_t>( new_tape_id );
        }
        // make sure tape_id value is valid for this thread
        CPPAD_ASSERT_UNKNOWN(
            size_t( *tape_id_p % CPPAD_MAX_NUM_THREADS ) == thread
        );
        // set the tape_id for this tape
        (*tape_h)->id_ = *tape_id_p;
    }
    // -----------------------------------------------------------------------
    // delete_tape_manage
    if( job == delete_tape_manage )
    {   // delete this tape
        CPPAD_ASSERT_UNKNOWN( *tape_h  != nullptr );
        delete *tape_h;
        *tape_h = nullptr;
        //
        // advance tape_id so that all AD<Base> variables become parameters
        CPPAD_ASSERT_KNOWN(
            std::numeric_limits<CPPAD_TAPE_ID_TYPE>::max()
            - CPPAD_MAX_NUM_THREADS > *tape_id_p,
            "To many different tapes given the type used for "
            "CPPAD_TAPE_ID_TYPE"
        );
        *tape_id_p  += CPPAD_MAX_NUM_THREADS;
    }
    // -----------------------------------------------------------------------
    return *tape_h;
}

/*!
Get a pointer to tape that records AD<Base> operations for the current thread.

\tparam Base
is the base type corresponding to AD<Base> operations.

\par thread
The current thread must be given by
\code
    thread = this->tape_id_ % CPPAD_MAX_NUM_THREADS
\endcode

\return
is a pointer to the tape that is currently recording AD<Base> operations
for the current thread.
This value must not be nullptr; i.e., there must be a tape currently
recording AD<Base> operations for this thread.
*/

template <class Base>
local::ADTape<Base> *AD<Base>::tape_this(void) const
{
    size_t thread = size_t( tape_id_ % CPPAD_MAX_NUM_THREADS );
    CPPAD_ASSERT_UNKNOWN( tape_id_ == *tape_id_ptr(thread) );
    CPPAD_ASSERT_UNKNOWN( *tape_handle(thread) != nullptr );
    return *tape_handle(thread);
}

} // END_CPPAD_NAMESPACE
# endif
