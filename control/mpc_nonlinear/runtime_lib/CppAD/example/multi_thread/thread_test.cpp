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
$begin thread_test.cpp$$
$spell
    inv
    mega
    cpp
    num
    pthread
    pthreads
    openmp
    bthread
    chkpoint
$$


$section Run Multi-Threading Examples and Speed Tests$$

$head Purpose$$
Runs the CppAD multi-threading examples and timing tests:

$head build$$
We use $icode build$$ for the directory where you run the $cref cmake$$
command.

$head threading$$
If the $cref cmake$$ command output indicates that
$code bthread$$, $code pthread$$, or $code openmp$$ is available,
you can run the program below with $icode threading$$ equal to
$code bthread$$, $code pthread$$, or $code openmp$$ respectively.

$head program$$
We use the notation $icode program$$ for
$codei%
     example_multi_thread_%threading%
%$$

$head Running Tests$$
You can build this program and run the default version of its test
parameters by executing the following commands:
$codei%
    cd %build%
    make check_%program%
%$$
After this operation, in the directory
$codei%
    %build%/example/multi_thread/%threading%
%$$
you can execute the following commands:
$codei%.
./%program% a11c
./%program% simple_ad
./%program% team_example
./%program% harmonic     %test_time% %max_threads% %mega_sum%
./%program% atomic_two   %test_time% %max_threads% %num_solve%
./%program% atomic_three %test_time% %max_threads% %num_solve%
./%program% chkpoint_one %test_time% %max_threads% %num_solve%
./%program% chkpoint_two %test_time% %max_threads% %num_solve%
./%program% multi_newton %test_time% %max_threads% \
    %num_zero% %num_sub% %num_sum% %use_ad%
%$$
We refer to the values $code a11c$$, ... , $code multi_newton$$
as the $icode test_case$$ below.

$children%
    example/multi_thread/openmp/a11c_openmp.cpp%
    example/multi_thread/bthread/a11c_bthread.cpp%
    example/multi_thread/pthread/a11c_pthread.cpp%

    example/multi_thread/openmp/simple_ad_openmp.cpp%
    example/multi_thread/bthread/simple_ad_bthread.cpp%
    example/multi_thread/pthread/simple_ad_pthread.cpp%

    example/multi_thread/team_example.cpp%
    example/multi_thread/harmonic.omh%
    example/multi_thread/multi_atomic_three.omh%
    example/multi_thread/multi_chkpoint_two.omh%
    example/multi_thread/multi_newton.omh%

    example/multi_thread/team_thread.hpp
%$$

$head a11c$$
The $icode test_case$$ $code a11c$$ runs the examples
$cref a11c_openmp.cpp$$,
$cref a11c_bthread.cpp$$, and
$cref a11c_pthread.cpp$$.
These cases demonstrate simple multi-threading,
without algorithmic differentiation, using
OpenMP, boost threads and pthreads respectively.

$head simple_ad$$
The $icode test_case$$ $code simple_ad$$ runs the examples
$cref simple_ad_openmp.cpp$$,
$cref simple_ad_bthread.cpp$$,
and
$cref simple_ad_pthread.cpp$$.
These cases demonstrate simple multi-threading,
with algorithmic differentiation, using
OpenMP, boost threads and pthreads respectively.

$head team_example$$
The $icode test_case$$ $code team_example$$ runs the
$cref team_example.cpp$$ example.
This case demonstrates simple multi-threading with algorithmic differentiation
and using a $cref/team of threads/team_thread.hpp/$$.

$head test_time$$
All of the other cases include the $icode test_time$$ argument.
This is the minimum amount of wall clock time that the test should take.
The number of repeats for the test will be increased until this time
is reached.
The reported time is the total wall clock time divided by the
number of repeats.

$subhead max_threads$$
All of the other cases include the $icode max_threads$$ argument.
This is a non-negative integer specifying
the maximum number of threads to use for the test.
The specified test is run with the following number of threads:
$codei%
    %num_threads% = 0 , %...% , %max_threads%
%$$
The value of zero corresponds to not using the multi-threading system.

$comment ------------------------------------------------------------------- $$

$head harmonic$$
The $icode test_case$$ $code harmonic$$ runs the
$cref harmonic_time$$ example.
This is a timing test for a multi-threading
example without algorithmic differentiation using a team of threads.

$subhead mega_sum$$
The command line argument $icode mega_sum$$
is an integer greater than or equal one and has the same meaning as in
$cref/harmonic_time/harmonic_time/mega_sum/$$.

$comment ------------------------------------------------------------------- $$

$head Atomic and Checkpoint$$
The $icode test_case$$ values
$code atomic_two$$,
$code atomic_three$$,
$code chkpoint_one$$,
$code chkpoint_two$$,
all run the same problem.
These cases preforms a timing test for a multi-threading
example without algorithmic differentiation using a team of threads.
$table
$icode test_case$$   $cnext Documentation                  $rnext
$code atomic_two$$   $cnext $cref multi_atomic_two.cpp$$   $rnext
$code atomic_three$$ $cnext $cref multi_atomic_three.cpp$$ $rnext
$code chkpoint_one$$ $cnext $cref multi_chkpoint_one.cpp$$ $rnext
$code chkpoint_two$$ $cnext $cref multi_chkpoint_two.cpp$$
$tend

$subhead num_solve$$
The command line argument $icode num_solve$$
is an integer specifying the number of solves; see
$cref/num_solve/multi_atomic_two_time/num_solve/$$ in $code multi_atomic_two_time$$.

$comment ------------------------------------------------------------------- $$

$head multi_newton$$
The $icode test_case$$ $code multi_newton$$  runs the
$cref multi_newton.cpp$$ example.
This preforms a timing test for a multi-threading
example with algorithmic differentiation using a team of threads.

$subhead num_zero$$
The command line argument $icode num_zero$$
is an integer greater than or equal two and has the same meaning as in
$cref/multi_newton_time/multi_newton_time/num_zero/$$.

$subhead num_sub$$
The command line argument $icode num_sub$$
is an integer greater than or equal one and has the same meaning as in
$cref/multi_newton_time/multi_newton_time/num_sub/$$.

$subhead num_sum$$
The command line argument $icode num_sum$$
is an integer greater than or equal one and has the same meaning as in
$cref/multi_newton_time/multi_newton_time/num_sum/$$.

$subhead use_ad$$
The command line argument $icode use_ad$$ is either
$code true$$ or $code false$$ and has the same meaning as in
$cref/multi_newton_time/multi_newton_time/use_ad/$$.

$comment ------------------------------------------------------------------- $$

$head Team Implementations$$
The following routines are used to implement the specific threading
systems through the common interface $cref team_thread.hpp$$:
$table
$rref team_openmp.cpp$$
$rref team_bthread.cpp$$
$rref team_pthread.cpp$$
$tend

$head Source$$
$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++

# include <cppad/cppad.hpp>
# include <cmath>
# include <cstring>
# include <ctime>
# include "team_thread.hpp"
# include "team_example.hpp"
# include "harmonic.hpp"
# include "multi_atomic_two.hpp"
# include "multi_atomic_three.hpp"
# include "multi_chkpoint_one.hpp"
# include "multi_chkpoint_two.hpp"
# include "multi_newton.hpp"

extern bool a11c(void);
extern bool simple_ad(void);

namespace {
    size_t arg2size_t(
        const char* arg       ,
        int limit             ,
        const char* error_msg )
    {   int i = std::atoi(arg);
        if( i >= limit )
            return size_t(i);
        std::cerr << "value = " << i << std::endl;
        std::cerr << error_msg << std::endl;
        exit(1);
    }
    double arg2double(
        const char* arg       ,
        double limit          ,
        const char* error_msg )
    {   double d = std::atof(arg);
        if( d >= limit )
            return d;
        std::cerr << "value = " << d << std::endl;
        std::cerr << error_msg << std::endl;
        exit(1);
    }
}

int main(int argc, char *argv[])
{   using CppAD::thread_alloc;
    bool ok         = true;
    using std::cout;
    using std::endl;

    // commnd line usage message
    const char* usage =
    "./<program> a11c\n"
    "./<program> simple_ad\n"
    "./<program> team_example\n"
    "./<program> harmonic     test_time max_threads mega_sum\n"
    "./<program> atomic_two   test_time max_threads num_solve\n"
    "./<program> atomic_three test_time max_threads num_solve\n"
    "./<program> chkpoint_one test_time max_threads num_solve\n"
    "./<program> chkpoint_two test_time max_threads num_solve\n"
    "./<program> multi_newton test_time max_threads \\\n"
    "   num_zero num_sub num_sum use_ad\\\n"
    "where <program> is example_multi_thread_<threading>\n"
    "and <threading> is bthread, openmp, or pthread";

    // command line argument values (assign values to avoid compiler warnings)
    size_t num_zero=0, num_sub=0, num_sum=0;
    bool use_ad=true;

    // put the date and time in the output file
    std::time_t rawtime;
    std::time( &rawtime );
    const char* gmt = std::asctime( std::gmtime( &rawtime ) );
    size_t len = size_t( std::strlen(gmt) );
    cout << "gmtime        = '";
    for(size_t i = 0; i < len; i++)
        if( gmt[i] != '\n' ) cout << gmt[i];
    cout << "';" << endl;

    // CppAD version number
    cout << "cppad_version = '" << CPPAD_PACKAGE_STRING << "';" << endl;

    // put the team name in the output file
    cout << "team_name     = '" << team_name() << "';" << endl;

    // print command line as valid matlab/octave
    cout << "command       = '" << argv[0];
    for(int i = 1; i < argc; i++)
        cout << " " << argv[i];
    cout << "';" << endl;

    ok = false;
    const char* test_name = "";
    if( argc > 1 )
        test_name = *++argv;
    bool run_a11c         = std::strcmp(test_name, "a11c")             == 0;
    bool run_simple_ad    = std::strcmp(test_name, "simple_ad")        == 0;
    bool run_team_example = std::strcmp(test_name, "team_example")     == 0;
    bool run_harmonic     = std::strcmp(test_name, "harmonic")         == 0;
    bool run_atomic_two   = std::strcmp(test_name, "atomic_two")       == 0;
    bool run_atomic_three = std::strcmp(test_name, "atomic_three")     == 0;
    bool run_chkpoint_one = std::strcmp(test_name, "chkpoint_one")     == 0;
    bool run_chkpoint_two = std::strcmp(test_name, "chkpoint_two")     == 0;
    bool run_multi_newton = std::strcmp(test_name, "multi_newton")     == 0;
    if( run_a11c || run_simple_ad || run_team_example )
        ok = (argc == 2);
    else if( run_harmonic
    || run_atomic_two
    || run_atomic_three
    || run_chkpoint_one
    || run_chkpoint_two )
        ok = (argc == 5);
    else if( run_multi_newton )
        ok = (argc == 8);
    if( ! ok )
    {   std::cerr << "test_name     = " << test_name << endl;
        std::cerr << "argc          = " << argc      << endl;
        std::cerr << usage << endl;
        exit(1);
    }
    if( run_a11c || run_simple_ad || run_team_example )
    {   if( run_a11c )
            ok        = a11c();
        else if( run_simple_ad )
            ok        = simple_ad();
        else
            ok        = team_example();
        if( thread_alloc::free_all() )
            cout << "free_all      = true;"  << endl;
        else
        {   ok = false;
            cout << "free_all      = false;" << endl;
        }
        if( ok )
            cout << "OK            = true;"  << endl;
        else cout << "OK            = false;" << endl;
        return ! ok;
    }

    // test_time
    double test_time = arg2double( *++argv, 0.,
        "run: test_time is less than zero"
    );

    // max_threads
    size_t max_threads = arg2size_t( *++argv, 0,
        "run: max_threads is less than zero"
    );

    size_t mega_sum  = 0; // assignment to avoid compiler warning
    size_t num_solve = 0;
    if( run_harmonic )
    {   // mega_sum
        mega_sum = arg2size_t( *++argv, 1,
            "run: mega_sum is less than one"
        );
    }
    else if( run_atomic_two
    || run_atomic_three
    || run_chkpoint_one
    || run_chkpoint_two )
    {   // num_solve
        num_solve = arg2size_t( *++argv, 1,
            "run: num_solve is less than one"
        );
    }
    else
    {   ok &= run_multi_newton;
        if( ! ok )
        {   cout << "thread_test: program error\n";
            return ! ok;
        }

        // num_zero
        num_zero = arg2size_t( *++argv, 2,
            "run: num_zero is less than two"
        );

        // num_sub
        num_sub = arg2size_t( *++argv, 1,
            "run: num_sub is less than one"
        );

        // num_sum
        num_sum = arg2size_t( *++argv, 1,
            "run: num_sum is less than one"
        );

        // use_ad
        ++argv;
        if( std::strcmp(*argv, "true") == 0 )
            use_ad = true;
        else if( std::strcmp(*argv, "false") == 0 )
            use_ad = false;
        else
        {   std::cerr << "run: use_ad = '" << *argv;
            std::cerr << "' is not true or false" << endl;
            exit(1);
        }
    }

    // run the test for each number of threads
    cout << "time_all  = [" << endl;
    for(size_t num_threads = 0; num_threads <= max_threads; num_threads++)
    {   double time_out;
        bool this_ok;

        // run the requested test
        if( run_harmonic ) this_ok = harmonic_time(
            time_out, test_time, num_threads, mega_sum
        );
        else if( run_atomic_two ) this_ok = multi_atomic_two_time(
            time_out, test_time, num_threads, num_solve
        );
        else if( run_atomic_three ) this_ok = multi_atomic_three_time(
            time_out, test_time, num_threads, num_solve
        );
        else if( run_chkpoint_one ) this_ok = multi_chkpoint_one_time(
            time_out, test_time, num_threads, num_solve
        );
        else if( run_chkpoint_two ) this_ok = multi_chkpoint_two_time(
            time_out, test_time, num_threads, num_solve
        );
        else
        {   assert( run_multi_newton);
            this_ok = multi_newton_time(
                time_out                ,
                test_time               ,
                num_threads             ,
                num_zero                ,
                num_sub                 ,
                num_sum                 ,
                use_ad
            );
        }
        // time_out
        cout << std::setw(20) << time_out << " % ";
        // num_threads
        if( num_threads == 0 )
            cout << "no threading";
        else
            cout << num_threads << " threads";
        if( this_ok )
            cout << " ok" << endl;
        else
            cout << " error" << endl;
        //
        ok &= this_ok;
    }
    cout << "];" << endl;
    //
    if( thread_alloc::free_all() )
        cout << "free_all      = true;"  << endl;
    else
    {   ok = false;
        cout << "free_all      = false;" << endl;
    }
    if( ok )
        cout << "OK            = true;"  << endl;
    else cout << "OK            = false;" << endl;

    return  ! ok;
}

// END C++
