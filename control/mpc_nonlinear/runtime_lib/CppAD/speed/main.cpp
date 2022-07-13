/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cstring>
# include <cstdlib>
# include <cassert>
# include <cstddef>
# include <iostream>
# include <iomanip>
# include <map>
# include <cppad/utility/vector.hpp>
# include <cppad/speed/det_grad_33.hpp>
# include <cppad/speed/det_33.hpp>
# include <cppad/speed/uniform_01.hpp>
# include <cppad/utility/poly.hpp>
# include <cppad/utility/track_new_del.hpp>
# include <cppad/utility/thread_alloc.hpp>

# ifdef CPPAD_ADOLC_SPEED
# define AD_PACKAGE "adolc"
# endif
# ifdef CPPAD_CPPAD_SPEED
# define AD_PACKAGE "cppad"
# endif
# ifdef CPPAD_DOUBLE_SPEED
# define AD_PACKAGE "double"
# endif
# ifdef CPPAD_FADBAD_SPEED
# define AD_PACKAGE "fadbad"
# endif
# ifdef CPPAD_PROFILE_SPEED
# define AD_PACKAGE "profile"
# endif
# ifdef CPPAD_SACADO_SPEED
# define AD_PACKAGE "sacado"
# endif
# ifdef CPPAD_CPPADCG_SPEED
# define AD_PACKAGE "cppadcg"
# endif
# ifdef CPPAD_XPACKAGE_SPEED
# define AD_PACKAGE "xpackage"
# endif

/*
$begin speed_main$$
$spell
    xpackage
    jac
    subgraph
    Jacobians
    hes
    subgraphs
    subsparsity
    revsparsity
    colpack
    onetape
    boolsparsity
    optionlist
    underbar
    alloc
    mat_mul
    retaped
    bool
    ddp
    cppad
    adolc
    fadbad
    sacado
    CppAD
    det
    lu
    Jacobian
    cppadcg
$$


$section Running the Speed Test Program$$

$head Syntax$$
$codei%speed/%package%/speed_%package% %test% %seed% %option_list%$$

$head Purpose$$
A version of this program runs the correctness tests
or the speed tests for one AD package identified by $icode package$$.

$head package$$

$subhead AD Package$$
The command line argument
$icode package$$ specifies one of the AD package.
The CppAD distribution comes with support for the following packages:
$cref/adolc/speed_adolc/$$,
$cref/cppad/speed_cppad/$$,
$cref/fadbad/speed_fadbad/$$,
$cref/sacado/speed_sacado/$$,
$cref/cppadcg/speed_cppadcg/$$.
You can extend this program to include other package;
see $cref speed_xpackage$$.

$subhead double$$
The value
$icode package$$ can be $code double$$ in which case
the function values (instead of derivatives) are computed
using double precision operations.
This enables one to compare the speed of computing function
values in $code double$$ to the speed of the derivative computations.
(It is often useful to divide the speed of the derivative computation by
the speed of the function evaluation in $code double$$.)

$subhead profile$$
In the special case where $icode package$$ is $code profile$$,
the CppAD package is compiled and run with profiling to aid in determining
where it is spending most of its time.

$head test$$
The argument $icode test$$ specifies which test to run
and has the following possible values:
$cref/correct/speed_main/test/correct/$$,
$cref/speed/speed_main/test/speed/$$,
$cref/det_minor/link_det_minor/$$,
$cref/det_lu/link_det_lu/$$,
$cref/mat_mul/link_mat_mul/$$,
$cref/ode/link_ode/$$,
$cref/poly/link_poly/$$,
$cref/sparse_hessian/link_sparse_hessian/$$,
$cref/sparse_jacobian/link_sparse_jacobian/$$.
You can experiment with changing the implementation of a
particular test for a particular package.

$subhead correct$$
If $icode test$$ is equal to $code correct$$,
all of the correctness tests are run.

$subhead speed$$
If $icode test$$ is equal to $code speed$$,
all of the speed tests are run.

$head seed$$
The command line argument $icode seed$$ is an unsigned integer
(all its characters are between 0 and 9).
The random number simulator $cref uniform_01$$ is initialized with
the call
$codei%
    uniform_01(%seed%)
%$$
before any of the testing routines (listed above) are called.

$head Global Options$$
This global variable has prototype
$srccode%cpp%
    extern std::map<std::string, bool> global_option;
%$$
The syntax
$codei%
    global_option["%option%"]
%$$
has the value true, if $icode option$$ is present,
and false otherwise.
This is true for each option that follows $icode seed$$.
The order of the options does not matter and the list can be empty.
Each option, is be a separate command line argument to the main program.
The documentation below specifics how the
$cref speed_cppad$$ program uses these options.
It is the intention that other packages use each option in a similar
way or make it invalid.
The implementation of each test should check that the option
setting are valid for that test and if not it should return false;
for example, see the source code for $cref adolc_sparse_hessian.cpp$$.

$subhead onetape$$
If this option is present,
$cref speed_cppad$$ will use one taping of the operation
sequence for all the repetitions of that speed test.
Otherwise, the
$cref/operation sequence/glossary/Operation/Sequence/$$
will be retaped for each test repetition.
$pre

$$
All of the tests, except $cref/det_lu/link_det_lu/$$,
have the same operation sequence for each repetition.
The operation sequence for $code det_lu$$
may be different because it depends on the matrix for which the determinant
is being calculated.
For this reason, $cref cppad_det_lu.cpp$$ returns false,
to indicate that the test not implemented,
when $code global_onetape$$ is true.

$subhead memory$$
This option is special because individual CppAD speed tests need not do
anything different if this option is true or false.
If the $code memory$$ option is present, the CppAD
$cref/hold_memory/ta_hold_memory/$$ routine will be called by
the speed test main program before any of the tests are executed
This should make the CppAD $code thread_alloc$$ allocator faster.
If it is not present, CppAD will used standard memory allocation.
Another package might use this option for a different
memory allocation method.

$subhead optimize$$
If this option is present,
CppAD will $cref optimize$$
the operation sequence before doing computations.
If it is false, this optimization will not be done.
Note that this option is usually slower unless it is combined with the
$code onetape$$ option.

$subhead atomic$$
If this option is present,
CppAD will use a user defined
$cref/atomic/atomic_two/$$ operation is used for the test.
So far, CppAD has only implemented
the $cref/mat_mul/link_mat_mul/$$ test as an atomic operation.

$subhead hes2jac$$
If this option is present,
$cref speed_cppad$$ will compute hessians as the Jacobian
of the gradient.
This is accomplished using
$cref/multiple levels/mul_level/$$ of AD.
So far, CppAD has only implemented
the $cref/sparse_hessian/link_sparse_hessian/$$
test in this manner.

$subhead subgraph$$
If this option is present,
$cref speed_cppad$$ will compute sparse Jacobians using subgraphs.
The CppAD $cref/sparse_jacobian/link_sparse_jacobian/$$
test is implemented for this option.
In addition, the CppAD $cref/sparse_hessian/link_sparse_hessian/$$
test is implemented for this option when $code hes2jac$$ is present.

$head Sparsity Options$$
The following options only apply to the
$cref/sparse_jacobian/link_sparse_jacobian/$$ and
$cref/sparse_hessian/link_sparse_hessian/$$ tests.
The other tests return false when any of these options
are present.

$subhead boolsparsity$$
If this option is present, CppAD will use a
$cref/vectors of bool/glossary/Sparsity Pattern/Boolean Vector/$$
to compute sparsity patterns.
Otherwise CppAD will use
$cref/vectors of sets/glossary/Sparsity Pattern/Vector of Sets/$$.

$subhead revsparsity$$
If this option is present,
CppAD will use reverse mode for to compute sparsity patterns.
Otherwise CppAD will use forward mode.

$subhead subsparsity$$
If this option is present,
CppAD will use subgraphs to compute sparsity patterns.
If
$code boolsparsity$$, $code revsparsity$$, or $code colpack$$ is also present,
the CppAD speed tests will return false; i.e., these options are not
supported by $cref subgraph_sparsity$$.

$subhead colpack$$
If this option is present,
CppAD will use $cref/colpack/colpack_prefix/$$ to do the coloring.
Otherwise, it will use it's own coloring algorithm.

$subhead symmetric$$
If this option is present, CppAD will use a symmetric
$cref/coloring method/sparse_hessian/work/color_method/$$
for computing Hessian sparsity patterns.
Otherwise, it will use a general coloring method.
The CppAD
$cref/sparse_hessian/link_sparse_hessian/$$ test
is implemented for this option.

$head Correctness Results$$
One, but not both, of the following two output lines
$codei%
    %package%_%test%_%optionlist%_available = false
    %package%_%test%_%optionlist%_ok = %flag%
%$$
is generated for each correctness test where
$icode package$$ and $icode test$$ are as above,
$icode optionlist$$ are the options (in $icode option_list$$)
separated by the underbar $code _$$ character
(whereas they are separated by spaces in $icode option_list$$),
and $icode flag$$ is $code true$$ or $code false$$.

$head Speed Results$$
For each speed test, corresponds to three lines of the
following form are generated:
$codei%
    %package%_%test%_%optionlist%_ok   = %flag%
    %test%_size = [ %size_1%, %...%, %size_n% ]
    %package%_%test%_rate = [ %rate_1%, %...%, %rate_n% ]
%$$
The values $icode package$$, $icode test$$, $icode optionlist$$,
and $icode flag$$ are as in the correctness results above.
The values $icode size_1$$, ..., $icode size_n$$ are the
size arguments used for the corresponding tests.
The values $icode rate_1$$, ..., $icode rate_n$$ are the number of times
per second that the corresponding size problem executed.

$subhead n_color$$
The $cref/sparse_jacobian/link_sparse_jacobian/$$
and $cref/sparse_hessian/link_sparse_hessian/$$ tests has an extra output
line with the following form
$codei%
    %package%_sparse_%test%_n_color = [ %n_color_1%, %...%, %n_color_n% ]
%$$
were $icode test$$ is $code jacobian$$ ($code hessian$$).
The values $icode n_color_1$$, ..., $icode n_color_n$$ are the number of
colors used for each sparse Jacobian (Hessian) calculation; see
$icode n_color$$ for
$cref/sparse_jac/sparse_jac/n_color/$$ and $icode n_sweep$$ for
$cref/sparse_hessian/sparse_hessian/n_sweep/$$.


$children%
    speed/src/link.omh
%$$
$head Link Routines$$
Each $cref/package/speed_main/package/$$
defines it's own version of one of the
$cref link_routines$$ listed below.
Each of these routines links this main program to the corresponding test:
$table
$rref link_det_lu$$
$rref link_det_minor$$
$rref link_mat_mul$$
$rref link_ode$$
$rref link_poly$$
$rref link_sparse_hessian$$
$rref link_sparse_jacobian$$
$tend


$end
-----------------------------------------------------------------------------
*/
// external routines

# define CPPAD_DECLARE_TIME(name)                         \
    extern bool available_##name(void);                   \
    extern bool correct_##name(bool is_package_double);   \
    extern double time_##name(double time_min, size_t size)

CPPAD_DECLARE_TIME(det_lu);
CPPAD_DECLARE_TIME(det_minor);
CPPAD_DECLARE_TIME(mat_mul);
CPPAD_DECLARE_TIME(ode);
CPPAD_DECLARE_TIME(poly);
CPPAD_DECLARE_TIME(sparse_hessian);
CPPAD_DECLARE_TIME(sparse_jacobian);
//
// some routines defined in src subdirectory
extern void info_sparse_jacobian(
    CppAD::vector<size_t>& size_vec    ,
    CppAD::vector<size_t>& n_color_vec
);
extern void info_sparse_hessian(size_t size, size_t& n_color);
//
// --------------------------------------------------------------------------
std::map<std::string, bool> global_option;
//
// If return value for the previous CppAD speed test was false, this is zero.
// Otherwise it is value returned by CppAD::thread_alloc::inuse for the
// current thread at end of the test.
size_t global_cppad_thread_alloc_inuse = 0;
//
// This is the value of seed in the main program comamnd line.
// It can be used by the sparse matrix routines to reset the random generator
// so same sparsity pattern is obtained during source generation and usage.
size_t global_seed= 0;
//
// --------------------------------------------------------------------------
namespace {
    using std::cout;
    using std::cerr;
    using std::endl;
    const char* option_list[] = {
        "memory",
        "onetape",
        "optimize",
        "atomic",
        "hes2jac",
        "subgraph",
        "boolsparsity",
        "revsparsity",
        "subsparsity",
        "colpack",
        "symmetric"
    };
    size_t num_option = sizeof(option_list) / sizeof( option_list[0] );
    // ----------------------------------------------------------------
    // not available test message
    void not_available_message(const char* test_name)
    {   cout << AD_PACKAGE << ": " << test_name;
        cout << " is not availabe with " << endl;
        int max_len = 0;
        for(size_t i = 0; i < num_option; i++)
        {   int len = int( std::strlen( option_list[i] ) );
            max_len = std::max( max_len, len);
        }
        for(size_t i = 0; i < num_option; i++)
        {   std::string option = option_list[i];
            if( global_option[option] )
                cout << std::setw(max_len + 1) << option << " = true\n";
            else
                cout << std::setw(max_len + 1) << option << " = false\n";
        }
    }
    // ------------------------------------------------------
    // output vector in form readable by octave or matlab
    // convert size_t to int to avoid warning by MS compiler
    void output(const CppAD::vector<size_t> &v)
    {   size_t i= 0, n = v.size();
        cout << "[ ";
        while(i < n)
        {   cout << int(v[i++]);
            if( i < n )
                cout << ", ";
        }
        cout << " ]";
    }

    // ----------------------------------------------------------------
    // function that runs one correctness case
    static size_t Run_ok_count    = 0;
    static size_t Run_error_count = 0;
    bool run_correct(
        bool available_case(void) ,
        bool correct_case(bool)   ,
        const char *case_name     )
    {   bool available = available_case();
        bool ok        = true;
        if( available )
        {
# ifdef CPPAD_DOUBLE_SPEED
            bool is_package_double = true;
# else
            bool is_package_double = false;
# endif
            ok = correct_case(is_package_double);
        }
        cout << AD_PACKAGE << "_" << case_name;
        for(size_t i = 0; i < num_option; i++)
        {   std::string option = option_list[i];
            if( global_option[option]  )
                cout << "_" << option;
        }
        if( ! available )
        {   cout << "_available = false" << endl;
            return ok;
        }
        cout << "_correct = ";
        if( ok )
        {   cout << " true" << endl;
            Run_ok_count++;
        }
        else
        {   cout << " false" << endl;
            Run_error_count++;
        }
        return ok;
    }
    // ----------------------------------------------------------------
    // function that runs one speed case
    void run_speed(
        double time_case(double time_min,  size_t size)  ,
        const CppAD::vector<size_t>&        size_vec     ,
        const std::string&                  case_name    )
    {   double time_min = 1.;
        cout << case_name << "_size = ";
        output(size_vec);
        cout << endl;
        cout << AD_PACKAGE << "_" << case_name << "_rate = ";
        cout << std::fixed;
        for(size_t i = 0; i < size_vec.size(); i++)
        {   if( i == 0 )
                cout << "[ ";
            else
                cout << ", ";
            cout << std::flush;
            size_t size = size_vec[i];
            double time = time_case(time_min, size);
            double rate = 1. / time;
            if( rate >= 1000 )
                cout << std::setprecision(0) << rate;
            else if( rate >= 10 )
                cout << std::setprecision(2) << rate;
            else
                cout << std::setprecision(4) << rate;
        }
        cout << " ]" << endl;
        //
        return;
    }
}

// main program that runs all the tests
int main(int argc, char *argv[])
{   bool ok = true;
    enum test_enum {
        test_correct,
        test_speed,
        test_det_lu,
        test_det_minor,
        test_mat_mul,
        test_ode,
        test_poly,
        test_sparse_hessian,
        test_sparse_jacobian,
        test_error
    };
    struct test_struct {
        const char       *name;
        const test_enum  index;
    };
    const test_struct test_list[]= {
        { "correct",            test_correct         },
        { "speed",              test_speed           },
        { "det_lu",             test_det_lu          },
        { "det_minor",          test_det_minor       },
        { "mat_mul",            test_mat_mul         },
        { "ode",                test_ode             },
        { "poly",               test_poly            },
        { "sparse_hessian",     test_sparse_hessian  },
        { "sparse_jacobian",    test_sparse_jacobian }
    };
    const size_t n_test  = sizeof(test_list) / sizeof(test_list[0]);
    //
    test_enum match = test_error;
    int    iseed = 0;
    bool   error = argc < 3;
    if( ! error )
    {   for(size_t i = 0; i < n_test; i++)
            if( strcmp(test_list[i].name, argv[1]) == 0 )
                match = test_list[i].index;
        error = match == test_error;
        for(size_t i = 0; *(argv[2] + i) != '\0'; ++i)
        {   error |= *(argv[2] + i) < '0';
            error |= '9' < *(argv[2] + i);
        }
        iseed = std::atoi( argv[2] );
        error |= iseed < 0;
        for(size_t i = 0; i < num_option; i++)
            global_option[ option_list[i] ] = false;
        for(size_t i = 3; i < size_t(argc); i++)
        {   bool found = false;
            for(size_t j = 0; j < num_option; j++)
            {   if( strcmp(argv[i], option_list[j]) == 0 )
                {   global_option[ option_list[j] ] = true;
                    found = true;
                }
            }
            error |= ! found;
        }
    }
    if( error )
    {   cout << "usage: ./speed_"
             << AD_PACKAGE << " test seed option_list" << endl;
        cout << "test choices:";
        for(size_t i = 0; i < n_test; i++)
        {   if( i % 5 == 0 )
                std::cout << "\n\t";
            else
                std::cout << ", ";
            cout << test_list[i].name;
        }
        cout << "\n\nseed: is a positive integer used as a random seed.";
        cout << "\n\noption_list: zero or more of the following:";
        for(size_t i = 0; i < num_option; i++)
        {   if( i % 5 == 0 )
                std::cout << "\n\t";
            else
                std::cout << ", ";
            cout << option_list[i];
        }
        cout << endl << endl;
        return 1;
    }
    if( global_option["memory"] )
        CppAD::thread_alloc::hold_memory(true);

    // initialize the random number simulator
    // (may be re-initialized by sparse jacobain test)
    global_seed = size_t(iseed);
    CppAD::uniform_01(global_seed);

    // arguments needed for speed tests
    size_t n_size   = 5;
    CppAD::vector<size_t> size_det_lu(n_size);
    CppAD::vector<size_t> size_det_minor(n_size);
    CppAD::vector<size_t> size_mat_mul(n_size);
    CppAD::vector<size_t> size_ode(n_size);
    CppAD::vector<size_t> size_poly(n_size);
    CppAD::vector<size_t> size_sparse_hessian(n_size);
    CppAD::vector<size_t> size_sparse_jacobian(n_size);
    for(size_t i = 0; i < n_size; i++)
    {   size_det_minor[i]   = i + 2;
        size_det_lu[i]      = 10 * i + 1;
        size_mat_mul[i]     = 10 * i + 1;
        size_ode[i]         = 10 * i + 1;
        size_poly[i]        = 10 * i + 1;
        size_sparse_hessian[i]  = 150 * (i + 1) * (i + 1);
        size_sparse_jacobian[i] = 150 * (i + 1) * (i + 1);
    }
    switch(match)
    {
        // run all the correctness tests
        case test_correct:
        ok &= run_correct( available_det_lu, correct_det_lu, "det_lu"
        );
        ok &= run_correct(
            available_det_minor, correct_det_minor, "det_minor"
        );
        ok &= run_correct(
            available_mat_mul, correct_mat_mul, "mat_mul"
        );
        ok &= run_correct(
            available_ode, correct_ode, "ode"
        );
        ok &= run_correct( available_poly, correct_poly, "poly"
        );
        ok &= run_correct(
            available_sparse_hessian,
            correct_sparse_hessian,
            "sparse_hessian"
        );
        ok &= run_correct(
            available_sparse_jacobian,
            correct_sparse_jacobian,
            "sparse_jacobian"
        );
        // summarize results
        assert( ok || (Run_error_count > 0) );
        if( ok )
        {   cout << "All " << int(Run_ok_count)
                 << " correctness tests passed." << endl;
        }
        else
        {   cout << int(Run_error_count)
                 << " correctness tests failed." << endl;
        }
        break;
        // ---------------------------------------------------------
        // run all the speed tests
        case test_speed:
        if( available_det_lu() ) run_speed(
            time_det_lu,          size_det_lu,          "det_lu"
        );
        if( available_det_minor() ) run_speed(
            time_det_minor,       size_det_minor,       "det_minor"
        );
        if( available_mat_mul() ) run_speed(
            time_mat_mul,           size_mat_mul,       "mat_mul"
        );
        if( available_ode() ) run_speed(
            time_ode,             size_ode,             "ode"
        );
        if( available_poly() ) run_speed(
            time_poly,            size_poly,            "poly"
        );
        if( available_sparse_hessian() ) run_speed(
            time_sparse_hessian,  size_sparse_hessian,  "sparse_hessian"
        );
        if( available_sparse_jacobian() ) run_speed(
        time_sparse_jacobian, size_sparse_jacobian, "sparse_jacobian"
        );
        ok = true;
        break;
        // ---------------------------------------------------------

        case test_det_lu:
        if( ! available_det_lu() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_det_lu, correct_det_lu, "det_lu")
        ;
        run_speed(time_det_lu,    size_det_lu,     "det_lu");
        break;
        // ---------------------------------------------------------

        case test_det_minor:
        if( ! available_det_minor() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_det_minor, correct_det_minor, "det_minor"
        );
        run_speed(time_det_minor, size_det_minor, "det_minor");
        break;
        // ---------------------------------------------------------

        case test_mat_mul:
        if( ! available_mat_mul() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_mat_mul, correct_mat_mul, "mat_mul"
        );
        run_speed(time_mat_mul, size_mat_mul, "mat_mul");
        break;
        // ---------------------------------------------------------

        case test_ode:
        if( ! available_ode() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_ode, correct_ode, "ode"
        );
        run_speed(time_ode,      size_ode,      "ode");
        break;
        // ---------------------------------------------------------

        case test_poly:
        if( ! available_poly() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_poly, correct_poly, "poly"
        );
        run_speed(time_poly,      size_poly,      "poly");
        break;
        // ---------------------------------------------------------

        case test_sparse_hessian:
        if( ! available_sparse_hessian() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_sparse_hessian,
            correct_sparse_hessian,
            "sparse_hessian"
        );
        run_speed(
            time_sparse_hessian, size_sparse_hessian,  "sparse_hessian"
        );
        cout << AD_PACKAGE << "_sparse_hessian_n_color = ";
        for(size_t i = 0; i < size_sparse_hessian.size(); i++)
        {   if( i == 0 )
                cout << "[ ";
            else
                cout << ", ";
            size_t n_color;
            info_sparse_hessian(size_sparse_hessian[i], n_color);
            cout << n_color;
        }
        cout << " ]" << endl;
        break;
        // ---------------------------------------------------------

        case test_sparse_jacobian:
        if( ! available_sparse_jacobian() )
        {   not_available_message( argv[1] );
            exit(1);
        }
        ok &= run_correct(
            available_sparse_jacobian,
            correct_sparse_jacobian,
            "sparse_jacobian"
        );
        run_speed(
            time_sparse_jacobian, size_sparse_jacobian, "sparse_jacobian"
        );
        {   // output values of n_color
            CppAD::vector<size_t> size_vec, n_color_vec;
            info_sparse_jacobian(size_vec, n_color_vec);
            assert( size_sparse_jacobian.size() == size_vec.size() );
            cout << AD_PACKAGE << "_sparse_jacobian_n_color = ";
            for(size_t i = 0; i < size_vec.size(); i++)
            {   assert( size_vec[i] == size_sparse_jacobian[i] );
                if( i == 0 )
                    cout << "[ ";
                else
                    cout << ", ";
                cout << n_color_vec[i];
            }
        }
        cout << " ]" << endl;
        break;
        // ---------------------------------------------------------

        default:
        assert(0);
    }
# ifndef NDEBUG
    // return memory for vectors that are still in scope
    size_det_lu.clear();
    size_det_minor.clear();
    size_mat_mul.clear();
    size_ode.clear();
    size_poly.clear();
    size_sparse_hessian.clear();
    size_sparse_jacobian.clear();
    // check for memory leak
    if( CppAD::thread_alloc::free_all() )
    {   Run_ok_count++;
        cout << "No memory leak detected" << endl;
    }
    else
    {   ok = false;
        Run_error_count++;
        cout << "Memory leak detected" << endl;
    }
# endif
    if( global_cppad_thread_alloc_inuse != 0 )
    {   cout << "memory allocated at end of last cppad speed test = ";
        cout << global_cppad_thread_alloc_inuse << std::endl;
    }
    if( ! ok )
    {   cout << "speed main: Error\n";
        exit(1);
    }
    std::cout << "speed main: OK\n";
    return 0;
}
