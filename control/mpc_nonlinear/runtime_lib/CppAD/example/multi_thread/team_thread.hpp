# ifndef CPPAD_EXAMPLE_MULTI_THREAD_TEAM_THREAD_HPP
# define CPPAD_EXAMPLE_MULTI_THREAD_TEAM_THREAD_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
$begin team_thread.hpp$$
$spell
    pthreads
    const
    cstddef
    bool
    pthread
    initializes
    hpp
    num
    CppAD
    ta
$$
$section Specifications for A Team of AD Threads$$


$head Syntax$$
$codei%include "team_thread.hpp"
%ok%   = team_create(%num_threads%)
%ok%   = team_work(%worker%)
%ok%   = team_destroy()
%name% = team_name()
%$$

$head Purpose$$
These routines start, use, and stop a team of threads that can
be used with the CppAD type $code AD<double>$$.
For example,
these could be OpenMP threads, pthreads, or Boost threads to name a few.

$head Restrictions$$
Calls to the routines
$code team_create$$,
$code team_work$$, and
$code team_destroy$$, must all be done by the master thread; i.e.,
$cref/thread_num/ta_thread_num/$$ must be zero.
In addition, they must all be done in sequential execution mode; i.e.,
when the master thread is the only thread that is running
($cref/in_parallel/ta_in_parallel/$$ must be false).

$head team_create$$
The argument
$icode%num_threads% > 0%$$ has type $code size_t$$
and specifies the number of threads in this team.
This initializes both $code AD<double>$$ and $code team_work$$
to be used with $icode num_threads$$.
If $icode%num_threads% > 1%$$,
$icode%num_threads% - 1%$$ new threads are created
and put in a waiting state until $code team_work$$ is called.

$head team_work$$
This routine may be called one or more times
between the call to $code team_create$$ and $code team_destroy$$.
The argument $icode worker$$ has type
$codei%bool %worker%(void)%$$.
Each call to $code team_work$$ runs $icode num_threads$$ versions
of $icode worker$$ with the corresponding value of
$cref/thread_num/ta_thread_num/$$
between zero and $icode%num_threads% - 1%$$ and
different for each thread,

$head team_destroy$$
This routine terminates all the other threads except for
thread number zero; i.e., it terminates the threads corresponding to
$codei%
    %thread_num% = 1 , ... , %num_threads%-1
%$$

$head team_name$$
This routines returns a name that identifies this thread_team.
The return value has prototype
$codei%
    const char* %name%
%$$
and is a statically allocated $code '\0'$$ terminated C string.

$head ok$$
The return value $icode ok$$ has type $code bool$$.
It is $code false$$ if an error is detected during the
corresponding call.
Otherwise it is $code true$$.

$children%
    example/multi_thread/openmp/team_openmp.cpp%
    example/multi_thread/bthread/team_bthread.cpp%
    example/multi_thread/pthread/team_pthread.cpp
%$$

$head Example Use$$
Example use of these specifications can be found in the file
$cref team_example.cpp$$.

$head Example Implementation$$
Example implementations of these specifications can be found in the files:
$table
$rref team_openmp.cpp$$
$rref team_bthread.cpp$$
$rref team_pthread.cpp$$
$tend

$head Speed Test of Implementation$$
Speed tests of using CppAD with the team implementations above
can be found in:
$table
$rref harmonic.cpp$$
$rref multi_newton.cpp$$
$tend

$head Source$$
$srccode%cpp% */
# include <cstddef> // for size_t

extern bool team_create(size_t num_threads);
extern bool team_work(void worker(void));
extern bool team_destroy(void);
extern const char* team_name(void);
/* %$$
$end
*/

# endif
