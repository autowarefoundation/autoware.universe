/* $Id: */

/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <assert.h>
# include <stdlib.h>
# include <math.h>
# include <stdio.h>
# include <stddef.h>
# include <float.h>

// In the case of plain C, we defined the type bool together with ture, false
# ifndef __cplusplus
# define bool int
# define true 1
# define false 0
# endif

/*
-------------------------------------------------------------------------------
$begin det_of_minor_c$$
$spell
    det
    const
$$

$section Determinant of a Minor$$

$head Syntax$$
$icode%d% = det_of_minor(%a%, %m%, %n%, %r%, %c%)%$$

$head Purpose$$
returns the determinant of a minor of the matrix $latex A$$
using expansion by minors.
The elements of the $latex n \times n$$ minor $latex M$$
of the matrix $latex A$$ are defined,
for $latex i = 0 , \ldots , n-1$$ and $latex j = 0 , \ldots , n-1$$, by
$latex \[
    M_{i,j} = A_{R(i), C(j)}
\]$$
where the functions
$latex R(i)$$ is defined by the $cref/argument r/det_of_minor/r/$$ and
$latex C(j)$$ is defined by the $cref/argument c/det_of_minor/c/$$.
$pre

$$
This function
is for example and testing purposes only.
Expansion by minors is chosen as an example because it uses
a lot of floating point operations yet does not require much source code
(on the order of $icode m$$ factorial floating point operations and
about 70 lines of source code including comments).
This is not an efficient method for computing a determinant;
for example, using an LU factorization would be better.

$head Determinant of A$$
If the following conditions hold, the minor is the
entire matrix $latex A$$ and hence $code det_of_minor$$
will return the determinant of $latex A$$:

$list number$$
$latex n = m$$.
$lnext
for $latex i = 0 , \ldots , m-1$$, $latex r[i] = i+1$$,
and $latex r[m] = 0$$.
$lnext
for $latex j = 0 , \ldots , m-1$$, $latex c[j] = j+1$$,
and $latex c[m] = 0$$.
$lend

$head a$$
The argument $icode a$$ has prototype
$codei%
    const double* %a%
%$$
and is a vector with size $latex m * m$$.
The elements of the $latex m \times m$$ matrix $latex A$$ are defined,
for $latex i = 0 , \ldots , m-1$$ and $latex j = 0 , \ldots , m-1$$, by
$latex \[
    A_{i,j} = a[ i * m + j]
\] $$

$head m$$
The argument $icode m$$ has prototype
$codei%
    size_t %m%
%$$
and is the size of the square matrix $latex A$$.

$head n$$
The argument $icode n$$ has prototype
$codei%
    size_t %n%
%$$
and is the size of the square minor $latex M$$.

$head r$$
The argument $icode r$$ has prototype
$codei%
    size_t* %r%
%$$
and is a vector with $latex m + 1$$ elements.
This vector defines the function $latex R(i)$$
which specifies the rows of the minor $latex M$$.
To be specific, the function $latex R(i)$$
for $latex i = 0, \ldots , n-1$$ is defined by
$latex \[
\begin{array}{rcl}
    R(0)   & = & r[m]
    \\
    R(i+1) & = & r[ R(i) ]
\end{array}
\] $$
All the elements of $icode r$$ must have value
less than or equal $icode m$$.
The elements of vector $icode r$$ are modified during the computation,
and restored to their original value before the return from
$code det_of_minor$$.

$head c$$
The argument $icode c$$ has prototype
$codei%
    size_t* %c%
%$$
and is a vector with $latex m + 1$$ elements
This vector defines the function $latex C(i)$$
which specifies the rows of the minor $latex M$$.
To be specific, the function $latex C(i)$$
for $latex j = 0, \ldots , n-1$$ is defined by
$latex \[
\begin{array}{rcl}
    C(0)   & = & c[m]
    \\
    C(j+1) & = & c[ C(j) ]
\end{array}
\] $$
All the elements of $icode c$$ must have value
less than or equal $icode m$$.
The elements of vector $icode c$$ are modified during the computation,
and restored to their original value before the return from
$code det_of_minor$$.

$head d$$
The result $icode d$$ has prototype
$codei%
    double %d%
%$$
and is equal to the determinant of the minor $latex M$$.

$spell
    Cj
$$
$head Source Code$$
$srccode%cpp% */
double det_of_minor(
    const double*        a  ,
    size_t               m  ,
    size_t               n  ,
    size_t*              r  ,
    size_t*              c  )
{   size_t R0, Cj, Cj1, j;
    double detM, M0j, detS;
    int s;

    R0 = r[m]; /* R(0) */
    Cj = c[m]; /* C(j)    (case j = 0) */
    Cj1 = m;   /* C(j-1)  (case j = 0) */

    /* check for 1 by 1 case */
    if( n == 1 ) return a[ R0 * m + Cj ];

    /* initialize determinant of the minor M */
    detM = 0.;

    /* initialize sign of factor for neat sub-minor */
    s = 1;

    /* remove row with index 0 in M from all the sub-minors of M */
    r[m] = r[R0];

    /* for each column of M */
    for(j = 0; j < n; j++)
    {   /* element with index (0,j) in the minor M */
        M0j = a[ R0 * m + Cj ];

        /* remove column with index j in M to form next sub-minor S of M */
        c[Cj1] = c[Cj];

        /* compute determinant of the current sub-minor S */
        detS = det_of_minor(a, m, n - 1, r, c);

        /* restore column Cj to representation of M as a minor of A */
        c[Cj1] = Cj;

        /* include this sub-minor term in the summation */
        if( s > 0 )
            detM = detM + M0j * detS;
        else
            detM = detM - M0j * detS;

        /* advance to neat column of M */
        Cj1 = Cj;
        Cj  = c[Cj];
        s   = - s;
    }

    /* restore row zero to the minor representation for M */
    r[m] = R0;

    /* return the determinant of the minor M */
    return detM;
}
/* %$$
$end
-------------------------------------------------------------------------------
$begin det_by_minor_c$$
$spell
    det
    const
$$

$section Compute Determinant using Expansion by Minors$$

$head Syntax$$
$icode%d% = det_by_minor(%a%, %n%)%$$

$head Purpose$$
returns the determinant of the matrix $latex A$$
using expansion by minors.
The elements of the $latex n \times n$$ minor $latex M$$
of the matrix $latex A$$ are defined,
for $latex i = 0 , \ldots , n-1$$ and $latex j = 0 , \ldots , n-1$$, by
$latex \[
    M_{i,j} = A_{i, j}
\]$$

$head a$$
The argument $icode a$$ has prototype
$codei%
    const double* %a%
%$$
and is a vector with size $latex m * m$$.
The elements of the $latex m \times m$$ matrix $latex A$$ are defined,
for $latex i = 0 , \ldots , m-1$$ and $latex j = 0 , \ldots , m-1$$, by
$latex \[
    A_{i,j} = a[ i * m + j]
\] $$

$head m$$
The argument $icode m$$ has prototype
$codei%
    size_t %m%
%$$
and is the number of rows (and columns) in the square matrix $latex A$$.

$hilitecmd%codep%$$
$hiliteseq%
    %det_of_minor%(%det_of_minor_c
%$$
$spell
    det
    malloc
    sizeof
$$
$head Source Code$$
$srccode%cpp% */
double det_by_minor(double* a, size_t m)
{   size_t *r, *c, i;
    double value;

    r = (size_t*) malloc( (m+1) * sizeof(size_t) );
    c = (size_t*) malloc( (m+1) * sizeof(size_t) );

    assert(m <= 100);
    for(i = 0; i < m; i++)
    {   r[i] = i+1;
        c[i] = i+1;
    }
    r[m] = 0;
    c[m] = 0;

    value = det_of_minor(a, m, m, r, c);

    free(r);
    free(c);
    return value;
}
/* %$$
$end
--------------------------------------------------------------------------
$begin uniform_01_c$$

$section Simulate a [0,1] Uniform Random Variate$$

$head Syntax$$
$codei%random_seed(%seed%)
%$$
$codei%uniform_01(%n%, %a%)%$$

$head Purpose$$
This routine is used to create random values for speed testing purposes.

$head seed$$
The argument $icode seed$$ has prototype
$codei%
    size_t %seed%
%$$
It specifies a seed
for the uniform random number generator.

$head n$$
The argument $icode n$$ has prototype
$codei%
    size_t %n%
%$$
It specifies the number of elements in the random vector $icode a$$.

$head a$$
The argument $icode a$$ has prototype
$codei%
    double* %a%
%$$.
The input value of the elements of $icode a$$ does not matter.
Upon return, the elements of $icode a$$ are set to values
randomly sampled over the interval [0,1].

$hilitecmd%codep%$$
$hiliteseq%
    %elapsed_seconds%(%elapsed_seconds_c%
    %repeat_det_by_minor%(%elapsed_seconds_c
%$$
$spell
    srand
$$
$head Source Code$$
$srccode%cpp% */
void random_seed(size_t seed)
{   srand( (unsigned int) seed );
}
void uniform_01(size_t n, double* a)
{   static double factor = 1. / (double) RAND_MAX;
    while(n--)
        a[n] = rand() * factor;
}
/* %$$
$end
------------------------------------------------------------------------------
$begin correct_det_by_minor_c$$
$spell
    det
    bool
$$

$section Correctness Test of det_by_minor Routine$$

$head Syntax$$
$icode%flag% = correct_det_by_minor()%$$

$head flag$$
The return value has prototype
$codei%
    bool %flag%
%$$
It value is $code 1$$ if the test passes and $code 0$$ otherwise.

$hilitecmd%codep%$$
$hiliteseq%
    %random_seed%(%uniform_01_c%
    %uniform_01%(%uniform_01_c
%$$
$spell
    fabs
$$
$head Source Code$$
$srccode%cpp% */
bool correct_det_by_minor(void)
{   double a[9], det, check;
    double eps99 = 99.0 * DBL_EPSILON;

    random_seed(123);
    uniform_01(9, a);

    /* compute determinant using expansion by minors */
    det = det_by_minor(a, 3);

    /* use expansion by minors to hand code the determinant  */
    check = 0.;
    check += a[0] * ( a[4] * a[8] - a[5] * a[7] );
    check -= a[1] * ( a[3] * a[8] - a[5] * a[6] );
    check += a[2] * ( a[3] * a[7] - a[4] * a[6] );

    if( fabs(det / check - 1.0) < eps99 )
        return true;
    return false;
}
/* %$$
$end
------------------------------------------------------------------------------
$begin repeat_det_by_minor_c$$
$spell
    det
$$

$section Repeat det_by_minor Routine A Specified Number of Times$$

$head Syntax$$
$codei%repeat_det_by_minor(%repeat%, %size%)%$$

$head repeat$$
The argument has prototype
$codei%
    size_t %repeat%
%$$
It specifies the number of times to repeat the calculation.

$head size$$
The argument has prototype
$codei%
    size_t %size%
%$$
It specifies the number of rows (and columns) in the square
matrix we are computing the determinant of.

$hilitecmd%codep%$$
$hiliteseq%
    %uniform_01%(%uniform_01_c%
    %det_by_minor%(%det_by_minor_c
%$$
$spell
    malloc
    sizeof
$$
$head Source Code$$
$srccode%cpp% */
void repeat_det_by_minor(size_t repeat, size_t size)
{   double *a;
    a = (double*) malloc( (size * size) * sizeof(double) );

    while(repeat--)
    {   uniform_01(size * size, a);
        det_by_minor(a, size);
    }

    free(a);
    return;
}
/* %$$
$end
------------------------------------------------------------------------------
$begin elapsed_seconds_c$$
$spell
    gettimeofday
$$

$section Returns Elapsed Number of Seconds$$


$head Syntax$$
$icode%s% = elapsed_seconds()%$$

$head Purpose$$
This routine is accurate to within .02 seconds
It does not necessary work for time intervals that are greater than a day.

$head s$$
is a $code double$$ equal to the
number of seconds since the first call to $code elapsed_seconds$$.

$spell
    Microsoft
    cassert
    milli
    sys
    endif
    usec
    diff
    bool
    struct
    timeval
$$
$head Source Code$$
$srccode%cpp% */
# if _MSC_VER
// ---------------------------------------------------------------------------
// Microsoft version of timer
# define NOMINMAX  // so windows.h does not define min and max as macros
# include <windows.h>
# include <cassert>
double elapsed_seconds(void)
{   static bool       first_  = true;
    static SYSTEMTIME st_;
    double hour, minute, second, milli, diff;
    SYSTEMTIME st;

    if( first_ )
    {   GetSystemTime(&st_);
        first_ = false;
        return 0.;
    }
    GetSystemTime(&st);

    hour   = (double) st.wHour         - (double) st_.wHour;
    minute = (double) st.wMinute       - (double) st_.wMinute;
    second = (double) st.wSecond       - (double) st_.wSecond;
    milli  = (double) st.wMilliseconds - (double) st_.wMilliseconds;

    diff   = 1e-3*milli + second + 60.*minute + 3600.*hour;
    if( diff < 0. )
        diff += 3600.*24.;
    assert( 0 <= diff && diff < 3600.*24. );

    return diff;
}
# else
// ---------------------------------------------------------------------------
// Unix version of timer
# include <sys/time.h>
double elapsed_seconds(void)
{   double sec, usec, diff;

    static bool first_ = true;
    static struct timeval tv_first;
    struct timeval        tv;
    if( first_ )
    {   gettimeofday(&tv_first, NULL);
        first_ = false;
        return 0.;
    }
    gettimeofday(&tv, NULL);
    assert( tv.tv_sec >= tv_first.tv_sec );

    sec  = (double)(tv.tv_sec -  tv_first.tv_sec);
    usec = (double)tv.tv_usec - (double)tv_first.tv_usec;
    diff = sec + 1e-6*usec;

    return diff;
}
# endif
/* %$$
$end
-----------------------------------------------------------------------------
$begin time_det_by_minor_c$$
$spell
    det
$$

$section Determine Amount of Time to Execute det_by_minor$$

$head Syntax$$
$icode%time% = time_test(%size%, %time_min%)%$$

$head Purpose$$
reports the amount of wall clock time for $code det_by_minor$$
to compute the determinant of a square matrix.

The $icode size$$ has prototype
$codei%
    size_t %size%
%$$
It specifies the number of rows (and columns) in the square matrix
that the determinant is being calculated for.

$head time_min$$
The argument $icode time_min$$ has prototype
$codei%
    double %time_min%
%$$
It specifies the minimum amount of time in seconds
that the $icode test$$ routine should take.
The calculations is repeated the necessary number of times so that
this amount of execution time (or more) is reached.

$head time$$
The return value $icode time$$ has prototype
$codei%
    double %time%
%$$
and is the number of wall clock seconds that it took for
$code det_by_minor$$ to compute its determinant
(plus overhead which includes choosing a random matrix).

$hilitecmd%codep%$$
$hiliteseq%
    %elapsed_seconds%(%elapsed_seconds_c%
     %repeat_det_by_minor%(%elapsed_seconds_c
%$$
$spell
    det
$$
$head Source Code$$
$srccode%cpp% */
double time_det_by_minor(size_t size, double time_min)
{   size_t repeat;
    double s0, s1, time;
    repeat = 0;
    s0     = elapsed_seconds();
    s1     = s0;
    while( s1 - s0 < time_min )
    {   if( repeat == 0 )
            repeat = 1;
        else
            repeat = 2 * repeat;
        s0     = elapsed_seconds();
        repeat_det_by_minor(repeat, size);
        s1     = elapsed_seconds();
    }
    time = (s1 - s0) / (double) repeat;
    return time;
}
/* %$$
$end
------------------------------------------------------------------------------
$begin main_compare_c$$

$section Main Program For Comparing C and C++ Speed$$

$hilitecmd%codep%$$
$hiliteseq%
    %correct_det_by_minor%(%correct_det_by_minor_c%
    %random_seed%(%uniform_01_c
%$$
$spell
    bool
    printf
    det
$$
$head Source Code$$
$srccode|cpp| */
int main(void)
{   bool flag;
    size_t i;

    random_seed(123);

    printf("correct_det_by_minor: ");
    flag = correct_det_by_minor();
    if( flag )
        printf("OK\n");
    else
        printf("Error\n");

    for(i = 0; i < 5; i++)
    {   double time_min = 1.0;
        size_t size     = 2 + i * 2;
        int   i_size    = (int) size;
        printf("time_det_minor for %d x %d matrix = ", i_size, i_size);
        printf("%g\n", time_det_by_minor(size, time_min) );
    }

    if( flag )
        return 0;
    return 1;
}
/* |$$
$end
*/
