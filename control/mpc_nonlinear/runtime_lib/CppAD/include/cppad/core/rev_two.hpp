# ifndef CPPAD_CORE_REV_TWO_HPP
# define CPPAD_CORE_REV_TWO_HPP
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
$begin RevTwo$$
$spell
    ddw
    typename
    Taylor
    const
$$





$section Reverse Mode Second Partial Derivative Driver$$

$head Syntax$$
$icode%ddw% = %f%.RevTwo(%x%, %i%, %j%)%$$


$head Purpose$$
We use $latex F : \B{R}^n \rightarrow \B{R}^m$$ to denote the
$cref/AD function/glossary/AD Function/$$ corresponding to $icode f$$.
The syntax above sets
$latex \[
    ddw [ k * p + \ell ]
    =
    \DD{ F_{i[ \ell ]} }{ x_{j[ \ell ]} }{ x_k } (x)
\] $$
for $latex k = 0 , \ldots , n-1$$
and $latex \ell = 0 , \ldots , p$$,
where $latex p$$ is the size of the vectors $icode i$$ and $icode j$$.

$head f$$
The object $icode f$$ has prototype
$codei%
    ADFun<%Base%> %f%
%$$
Note that the $cref ADFun$$ object $icode f$$ is not $code const$$
(see $cref/RevTwo Uses Forward/RevTwo/RevTwo Uses Forward/$$ below).

$head x$$
The argument $icode x$$ has prototype
$codei%
    const %BaseVector% &%x%
%$$
(see $cref/BaseVector/RevTwo/BaseVector/$$ below)
and its size
must be equal to $icode n$$, the dimension of the
$cref/domain/fun_property/Domain/$$ space for $icode f$$.
It specifies
that point at which to evaluate the partial derivatives listed above.

$head i$$
The argument $icode i$$ has prototype
$codei%
    const %SizeVector_t% &%i%
%$$
(see $cref/SizeVector_t/RevTwo/SizeVector_t/$$ below)
We use $icode p$$ to denote the size of the vector $icode i$$.
All of the indices in $icode i$$
must be less than $icode m$$, the dimension of the
$cref/range/fun_property/Range/$$ space for $icode f$$; i.e.,
for $latex \ell = 0 , \ldots , p-1$$, $latex i[ \ell ]  < m$$.

$head j$$
The argument $icode j$$ has prototype
$codei%
    const %SizeVector_t% &%j%
%$$
(see $cref/SizeVector_t/RevTwo/SizeVector_t/$$ below)
and its size must be equal to $icode p$$,
the size of the vector $icode i$$.
All of the indices in $icode j$$
must be less than $icode n$$; i.e.,
for $latex \ell = 0 , \ldots , p-1$$, $latex j[ \ell ]  < n$$.

$head ddw$$
The result $icode ddw$$ has prototype
$codei%
    %BaseVector% %ddw%
%$$
(see $cref/BaseVector/RevTwo/BaseVector/$$ below)
and its size is $latex n * p$$.
It contains the requested partial derivatives; to be specific,
for $latex k = 0 , \ldots , n - 1 $$
and $latex \ell = 0 , \ldots , p - 1$$
$latex \[
    ddw [ k * p + \ell ]
    =
    \DD{ F_{i[ \ell ]} }{ x_{j[ \ell ]} }{ x_k } (x)
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

$head RevTwo Uses Forward$$
After each call to $cref Forward$$,
the object $icode f$$ contains the corresponding
$cref/Taylor coefficients/glossary/Taylor Coefficient/$$.
After a call to $code RevTwo$$,
the zero order Taylor coefficients correspond to
$icode%f%.Forward(0, %x%)%$$
and the other coefficients are unspecified.

$head Examples$$
$children%
    example/general/rev_two.cpp
%$$
The routine
$cref/RevTwo/rev_two.cpp/$$ is both an example and test.
It returns $code true$$, if it succeeds and $code false$$ otherwise.

$end
-----------------------------------------------------------------------------
*/

//  BEGIN CppAD namespace
namespace CppAD {

template <class Base, class RecBase>
template <class BaseVector, class SizeVector_t>
BaseVector ADFun<Base,RecBase>::RevTwo(
    const BaseVector   &x,
    const SizeVector_t &i,
    const SizeVector_t &j)
{   size_t i1;
    size_t j1;
    size_t k;
    size_t l;

    size_t n = Domain();
    size_t m = Range();
    size_t p = i.size();

    // check BaseVector is Simple Vector class with Base elements
    CheckSimpleVector<Base, BaseVector>();

    // check SizeVector_t is Simple Vector class with size_t elements
    CheckSimpleVector<size_t, SizeVector_t>();

    CPPAD_ASSERT_KNOWN(
        x.size() == n,
        "RevTwo: Length of x not equal domain dimension for f."
    );
    CPPAD_ASSERT_KNOWN(
        i.size() == j.size(),
        "RevTwo: Lenght of the i and j vectors are not equal."
    );
    // point at which we are evaluating the second partials
    Forward(0, x);

    // dimension the return value
    BaseVector ddw(n * p);

    // direction vector in argument space
    BaseVector dx(n);
    for(j1 = 0; j1 < n; j1++)
        dx[j1] = Base(0.0);

    // direction vector in range space
    BaseVector w(m);
    for(i1 = 0; i1 < m; i1++)
        w[i1] = Base(0.0);

    // place to hold the results of a reverse calculation
    BaseVector r(n * 2);

    // check the indices in i and j
    for(l = 0; l < p; l++)
    {   i1 = i[l];
        j1 = j[l];
        CPPAD_ASSERT_KNOWN(
        i1 < m,
        "RevTwo: an eleemnt of i not less than range dimension for f."
        );
        CPPAD_ASSERT_KNOWN(
        j1 < n,
        "RevTwo: an element of j not less than domain dimension for f."
        );
    }

    // loop over all forward directions
    for(j1 = 0; j1 < n; j1++)
    {   // first order forward mode calculation done
        bool first_done = false;
        for(l = 0; l < p; l++) if( j[l] == j1 )
        {   if( ! first_done )
            {   first_done = true;

                // first order forward mode in j1 direction
                dx[j1] = Base(1.0);
                Forward(1, dx);
                dx[j1] = Base(0.0);
            }
            // execute a reverse in this component direction
            i1    = i[l];
            w[i1] = Base(1.0);
            r     = Reverse(2, w);
            w[i1] = Base(0.0);

            // place the reverse result in return value
            for(k = 0; k < n; k++)
                ddw[k * p + l] = r[k * 2 + 1];
        }
    }
    return ddw;
}

} // END CppAD namespace

# endif
