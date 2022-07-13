# ifndef CPPAD_CORE_FOR_TWO_HPP
# define CPPAD_CORE_FOR_TWO_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
$begin ForTwo$$
$spell
    ddy
    typename
    Taylor
    const
$$





$section Forward Mode Second Partial Derivative Driver$$

$head Syntax$$
$icode%ddy% = %f%.ForTwo(%x%, %j%, %k%)%$$


$head Purpose$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to $icode f$$.
The syntax above sets
$latex \[
    ddy [ i * p + \ell ]
    =
    \DD{ F_i }{ x_{j[ \ell ]} }{ x_{k[ \ell ]} } (x)
\] $$
for $latex i = 0 , \ldots , m-1$$
and $latex \ell = 0 , \ldots , p$$,
where $latex p$$ is the size of the vectors $icode j$$ and $icode k$$.

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
Note that the $cref ADFun$$ object $icode f$$ is not $code const$$
(see $cref/ForTwo Uses Forward/ForTwo/ForTwo Uses Forward/$$ below).

$head x$$
The argument $icode x$$ has prototype
$codei%
    const %BaseVector% &%x%
%$$
(see $cref/BaseVector/ForTwo/BaseVector/$$ below)
and its size
must be equal to $icode n$$, the dimension of the
$cref/domain/fun_property/Domain/$$ space for $icode f$$.
It specifies
that point at which to evaluate the partial derivatives listed above.

$head j$$
The argument $icode j$$ has prototype
$codei%
    const %SizeVector_t% &%j%
%$$
(see $cref/SizeVector_t/ForTwo/SizeVector_t/$$ below)
We use $icode p$$ to denote the size of the vector $icode j$$.
All of the indices in $icode j$$
must be less than $icode n$$; i.e.,
for $latex \ell = 0 , \ldots , p-1$$, $latex j[ \ell ]  < n$$.

$head k$$
The argument $icode k$$ has prototype
$codei%
    const %SizeVector_t% &%k%
%$$
(see $cref/SizeVector_t/ForTwo/SizeVector_t/$$ below)
and its size must be equal to $icode p$$,
the size of the vector $icode j$$.
All of the indices in $icode k$$
must be less than $icode n$$; i.e.,
for $latex \ell = 0 , \ldots , p-1$$, $latex k[ \ell ]  < n$$.

$head ddy$$
The result $icode ddy$$ has prototype
$codei%
    %BaseVector% %ddy%
%$$
(see $cref/BaseVector/ForTwo/BaseVector/$$ below)
and its size is $latex m * p$$.
It contains the requested partial derivatives; to be specific,
for $latex i = 0 , \ldots , m - 1 $$
and $latex \ell = 0 , \ldots , p - 1$$
$latex \[
    ddy [ i * p + \ell ]
    =
    \DD{ F_i }{ x_{j[ \ell ]} }{ x_{k[ \ell ]} } (x)
\] $$

$head BaseVector$$
The type $icode BaseVector$$ must be a $cref SimpleVector$$ class with
$cref/elements of type Base/SimpleVector/Elements of Specified Type/$$.
The routine $cref CheckSimpleVector$$ will generate an error message
if this is not the case.

$head SizeVector_t$$
The type $icode SizeVector_t$$ must be a $cref SimpleVector$$ class with
$cref/elements of type size_t/SimpleVector/Elements of Specified Type/$$.
The routine $cref CheckSimpleVector$$ will generate an error message
if this is not the case.

$head ForTwo Uses Forward$$
After each call to $cref Forward$$,
the object $icode f$$ contains the corresponding
$cref/Taylor coefficients/glossary/Taylor Coefficient/$$.
After a call to $code ForTwo$$,
the zero order Taylor coefficients correspond to
$icode%f%.Forward(0, %x%)%$$
and the other coefficients are unspecified.

$head Examples$$
$children%
    example/general/for_two.cpp
%$$
The routine
$cref/ForTwo/for_two.cpp/$$ is both an example and test.
It returns $code true$$, if it succeeds and $code false$$ otherwise.

$end
-----------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base, class RecBase>
template <class BaseVector, class SizeVector_t>
BaseVector ADFun<Base,RecBase>::ForTwo(
    const BaseVector   &x,
    const SizeVector_t &j,
    const SizeVector_t &k)
{   size_t i;
    size_t j1;
    size_t k1;
    size_t l;

    size_t n = Domain();
    size_t m = Range();
    size_t p = j.size();

    // check BaseVector is Simple Vector class with Base type elements
    CheckSimpleVector<Base, BaseVector>();

    // check SizeVector_t is Simple Vector class with size_t elements
    CheckSimpleVector<size_t, SizeVector_t>();

    CPPAD_ASSERT_KNOWN(
        x.size() == n,
        "ForTwo: Length of x not equal domain dimension for f."
    );
    CPPAD_ASSERT_KNOWN(
        j.size() == k.size(),
        "ForTwo: Lenght of the j and k vectors are not equal."
    );
    // point at which we are evaluating the second partials
    Forward(0, x);


    // dimension the return value
    BaseVector ddy(m * p);

    // allocate memory to hold all possible diagonal Taylor coefficients
    // (for large sparse cases, this is not efficient)
    BaseVector D(m * n);

    // boolean flag for which diagonal coefficients are computed
    CppAD::vector<bool> c(n);
    for(j1 = 0; j1 < n; j1++)
        c[j1] = false;

    // direction vector in argument space
    BaseVector dx(n);
    for(j1 = 0; j1 < n; j1++)
        dx[j1] = Base(0.0);

    // result vector in range space
    BaseVector dy(m);

    // compute the diagonal coefficients that are needed
    for(l = 0; l < p; l++)
    {   j1 = j[l];
        k1 = k[l];
        CPPAD_ASSERT_KNOWN(
        j1 < n,
        "ForTwo: an element of j not less than domain dimension for f."
        );
        CPPAD_ASSERT_KNOWN(
        k1 < n,
        "ForTwo: an element of k not less than domain dimension for f."
        );
        size_t count = 2;
        while(count)
        {   count--;
            if( ! c[j1] )
            {   // diagonal term in j1 direction
                c[j1]  = true;
                dx[j1] = Base(1.0);
                Forward(1, dx);

                dx[j1] = Base(0.0);
                dy     = Forward(2, dx);
                for(i = 0; i < m; i++)
                    D[i * n + j1 ] = dy[i];
            }
            j1 = k1;
        }
    }
    // compute all the requested cross partials
    for(l = 0; l < p; l++)
    {   j1 = j[l];
        k1 = k[l];
        if( j1 == k1 )
        {   for(i = 0; i < m; i++)
                ddy[i * p + l] = Base(2.0) * D[i * n + j1];
        }
        else
        {
            // cross term in j1 and k1 directions
            dx[j1] = Base(1.0);
            dx[k1] = Base(1.0);
            Forward(1, dx);

            dx[j1] = Base(0.0);
            dx[k1] = Base(0.0);
            dy = Forward(2, dx);

            // place result in return value
            for(i = 0; i < m; i++)
                ddy[i * p + l] = dy[i] - D[i*n+j1] - D[i*n+k1];

        }
    }
    return ddy;
}

} // END CppAD namespace

# endif
