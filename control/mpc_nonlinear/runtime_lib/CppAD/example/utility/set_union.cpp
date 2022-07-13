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
$begin set_union.cpp$$

$section Set Union: Example and Test$$


$srcthisfile%0%// BEGIN C++%// END C++%1%$$

$end
*/
// BEGIN C++
# include <cppad/utility/set_union.hpp>

bool set_union(void)
{   bool ok = true;

    // create empty sets
    std::set<size_t> left, right, result;

    // set left = {1, 2}
    left.insert(1);
    left.insert(2);

    // set right = {2, 3}
    right.insert(2);
    right.insert(3);

    // set result = {1, 2} U {2, 3}
    result = CppAD::set_union(left, right);

    // expected result
    size_t check_vec[] = {1, 2, 3};
    std::set<size_t> check_set(check_vec, check_vec + 3);

    // check result
    ok &= result == check_set;

    return ok;
}

// END C++
