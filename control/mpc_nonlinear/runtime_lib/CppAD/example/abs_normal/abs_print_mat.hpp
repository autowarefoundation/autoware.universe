# ifndef CPPAD_EXAMPLE_ABS_NORMAL_ABS_PRINT_MAT_HPP
# define CPPAD_EXAMPLE_ABS_NORMAL_ABS_PRINT_MAT_HPP
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
$begin abs_print_mat$$
$spell
    nr
    nc
    std::cout
$$
$section abs_normal: Print a Vector or Matrix$$

$head Syntax$$
$codei%abs_print_mat(%name%, %nr%, %nc%, %mat%)%$$

$head Prototype$$
$srcthisfile%
    0%// BEGIN PROTOTYPE%// END PROTOTYPE%
1%$$


$head Purpose$$
This routine is used by the $cref/abs_normal/example_abs_normal/$$ examples to print
vectors and matrices.
A new-line is printed at the end of this output.

$head name$$
This is a name that is printed before the vector or matrix.

$head nr$$
This is the number of rows in the matrix. Use $icode%nr% = 1%$$ for
row vectors.

$head nc$$
This is the number of columns in the matrix. Use $icode%nc% = 1%$$ for
column vectors.

$head mat$$
This is a
$cref/row-major/glossary/Row-major Representation/$$ representation
of the matrix (hence a $cref SimpleVector$$).
The syntax
$codei%
    std::cout << %mat%[%i%]%
%$$
must output the $th i$$ element of the simple vector $icode mat$$.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/cppad.hpp>

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

// BEGIN PROTOTYPE
template <class Vector>
void abs_print_mat(
    const std::string& name ,
    size_t             nr   ,
    size_t             nc   ,
    const Vector&      mat  )
// END PROTOTYPE
{
    CPPAD_ASSERT_KNOWN(
        size_t(mat.size()) == nr * nc,
        "abs_print_mat: size of mat is not nr * nc"
    );
    // output name
    std::cout << name << " =";
    //
    // handel empty case
    if( nr == 0 || nc == 0 )
    {   std::cout << " " << nr << " by " << nc << " empty matrix\n";
        return;
    }
    //
    // handle vector case
    if( nr == 1 || nc == 1 )
    {   std::cout << " [";
        for(size_t i = 0; i < nr * nc; i++)
        {   if( i > 0 )
                std::cout << ", ";
            std::cout << mat[i];
        }
        std::cout << "]";
        //
        // column vectors are printed as row vectors with a transpose at end
        if( nr > 1 )
            std::cout << "^T";
        //
        std::cout << "\n";
        return;
    }
    // non-empty matrix
    std::cout << "\n";
    for(size_t i = 0; i < nr; i++)
    {   std::cout << "[";
        for(size_t j = 0; j < nc; j++)
        {   if( j > 0 )
                std::cout << ", ";
            std::cout << mat[i * nc + j];
        }
        std::cout << "]\n";
    }
    return;
}

} // END_CPPAD_NAMESPACE
# endif
