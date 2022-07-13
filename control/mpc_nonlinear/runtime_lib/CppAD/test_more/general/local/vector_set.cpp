/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <cppad/cppad.hpp>

namespace { //  BEGIN empty namespace

template <class SetVector>
bool test_no_other(void)
{   bool ok = true;

    SetVector vec_set;
    size_t n_set = 4;
    size_t end   = 5;
    //
    // set size of vec_set
    vec_set.resize(n_set, end);
    ok &= end == vec_set.end();
    ok &= n_set == vec_set.n_set();
    //
    // test resizing to zero
    vec_set.resize(0, 0);
    ok &= 0 == vec_set.n_set();
    ok &= 0 == vec_set.end();
    //
    // set size of vec_set
    vec_set.resize(n_set, end);
    ok &= end == vec_set.end();
    ok &= n_set == vec_set.n_set();

    //
    // add the element i+1 to set i
    for(size_t i = 1; i < n_set; i++)
        vec_set.add_element(i, i+1);
    //
    // check for element i and i+1 in set i
    for(size_t i = 0; i < n_set; i++)
    {   ok &= ! vec_set.is_element(i, i);
        if( i == 0 )
            ok &=  ! vec_set.is_element(i, i+1);
        else
            ok &=  vec_set.is_element(i, i+1);
    }
    //
    // set an empty set to value of set 2
    size_t target = 0;
    size_t source = 2;
    vec_set.assignment(target, source, vec_set);
    ok &= ! vec_set.is_element(target, source);
    ok &= vec_set.is_element(target, source+1);
    //
    // set a non-empty set to the value of set 2
    target = 1;
    vec_set.assignment(target, source, vec_set);
    ok &= ! vec_set.is_element(target, source);
    ok &= vec_set.is_element(target, source+1);
    //
    // add an element to set 1, one of the three vectors equal to set 2
    target = 1;
    vec_set.add_element(target, source);
    ok &= vec_set.is_element(target, source);
    ok &= vec_set.is_element(target, source+1);
    ok &= ! vec_set.is_element(source, source);
    ok &= vec_set.is_element(source, source+1);
    //
    // now take the union of set 2 and set 3 and place in set 0
    // (wich is sharing a list with set 2)
    target = 0;
    vec_set.binary_union(target, source, source+1, vec_set);
    ok &= vec_set.is_element(target, source+1);
    ok &= vec_set.is_element(target, source+2);
    ok &= vec_set.is_element(source, source+1);
    ok &= ! vec_set.is_element(source, source+2);
    //
    // now check the elements in set 0 by iterating over them
    typename SetVector::const_iterator itr(vec_set, target);
    ok &= *itr     == source+1;
    ok &= *(++itr) == source+2;
    ok &= *(++itr) == end;
    //
    // now test clear
    vec_set.clear(1);
    ok &= ! vec_set.is_element(1, source+1);
    ok &= vec_set.is_element(0, source+1);
    //
    // now force list_setvec garbage collection by setting all sets
    // equal to set 0
    for(size_t i = 1; i < n_set; i++)
    {   vec_set.assignment(i, 0, vec_set);
        ok &= vec_set.is_element(i, source+1);
        ok &= vec_set.is_element(i, source+2);
    }
    //
    return ok;
}

template <class SetVector>
bool test_yes_other(void)
{   bool ok = true;

    SetVector vec_set, other_vec;
    size_t n_set = 4;
    size_t end   = 5;
    vec_set.resize(n_set, end);
    other_vec.resize(n_set, end);
    //
    // add element i to set i in vec_set
    // add element i+1 to set i in other
    for(size_t i = 1; i < n_set; i++)
    {   vec_set.add_element(i, i);
        other_vec.add_element(i, i+1);
    }
    //
    // assignment of one set from other
    size_t target = 0;
    size_t source = 1;
    vec_set.assignment(target, source, other_vec);
    ok &= ! vec_set.is_element(target, source);
    ok &= vec_set.is_element(target, source+1);
    //
    // now take the union of a set from vec_set and from other_vec
    target       = 2; // where result goes in vec_set
    size_t left  = 2; // left operand in vec_set
    size_t right = 2; // right operand in other
    vec_set.binary_union(target, left, right, other_vec);
    ok &= vec_set.is_element(target, left);
    ok &= vec_set.is_element(target, right+1);
    //
    // now use assignment for entire vector of sets
    vec_set = other_vec;
    ok &= ! vec_set.is_element(0, 0);
    ok &= ! vec_set.is_element(0, 1);
    for(size_t i = 1; i < n_set; i++)
    {   ok &= ! vec_set.is_element(i, i);
        ok &=   vec_set.is_element(i, i+1);
    }
    return ok;
}

template <class SetVector>
bool test_intersection(void)
{   bool ok = true;
    //
    SetVector vec_set;
    size_t n_set = 3;
    size_t end   = 5;
    vec_set.resize(n_set, end);
    //
    // set[0] = {1, 2}
    vec_set.add_element(0, 1);
    vec_set.add_element(0, 2);
    //
    // set[1] = {2, 3}
    vec_set.add_element(1, 2);
    vec_set.add_element(1, 3);
    //
    // set[2] = set[0] intersect set[1]
    size_t target = 2;
    size_t left   = 0;
    size_t right  = 1;
    vec_set.binary_intersection(target, left, right, vec_set);
    //
    typename SetVector::const_iterator itr1(vec_set, target);
    ok &= *itr1     == 2;
    ok &= *(++itr1) == end;
    //
    // other[1] = set[1]
    SetVector other;
    other.resize(n_set, end);
    target        = 1;
    size_t source = 1;
    other.assignment(target, source, vec_set);
    //
    // set[2] = set[0] intersect other[1]
    target = 2;
    left   = 0;
    right  = 1;
    vec_set.binary_intersection(target, left, right, other);
    //
    typename SetVector::const_iterator itr2(vec_set, target);
    ok &= *itr2     == 2;
    ok &= *(++itr2) == end;
    //
    return ok;
}

template<class SetVector>
bool test_post(void)
{   bool ok = true;
    //
    SetVector vec_set;
    size_t n_set = 3;
    size_t end   = 5;
    vec_set.resize(n_set, end);
    //
    // set[1] = {1, 2}
    vec_set.add_element(1, 1);
    vec_set.add_element(1, 2);
    //
    // set[1] = {1, 2} union (2, 4, 4)  = {1, 2, 4}
    size_t target = 1;
    vec_set.post_element(target, 2);
    vec_set.post_element(target, 4);
    vec_set.post_element(target, 4);
    vec_set.process_post(target);
    //
    typename SetVector::const_iterator itr1(vec_set, target);
    ok &= *itr1     == 1;
    ok &= *(++itr1) == 2;
    ok &= *(++itr1) == 4;
    ok &= *(++itr1) == end;
    //
    // set[1] = {1, 2, 4} union (1, 2)
    target = 1;
    vec_set.post_element(target, 1);
    vec_set.post_element(target, 2);
    vec_set.process_post(target);
    //
    typename SetVector::const_iterator itr2(vec_set, target);
    ok &= *itr2     == 1;
    ok &= *(++itr2) == 2;
    ok &= *(++itr2) == 4;
    ok &= *(++itr2) == end;
    //
    return ok;
}

} // END empty namespace

bool vector_set(void)
{   bool ok = true;
    //
    ok     &= test_no_other<CppAD::local::sparse::pack_setvec>();
    ok     &= test_no_other<CppAD::local::sparse::list_setvec>();
    ok     &= test_no_other<CppAD::local::sparse::svec_setvec>();
    //
    ok     &= test_yes_other<CppAD::local::sparse::pack_setvec>();
    ok     &= test_yes_other<CppAD::local::sparse::list_setvec>();
    ok     &= test_yes_other<CppAD::local::sparse::svec_setvec>();
    //
    ok     &= test_intersection<CppAD::local::sparse::pack_setvec>();
    ok     &= test_intersection<CppAD::local::sparse::list_setvec>();
    ok     &= test_intersection<CppAD::local::sparse::svec_setvec>();
    //
    ok     &= test_post<CppAD::local::sparse::pack_setvec>();
    ok     &= test_post<CppAD::local::sparse::list_setvec>();
# ifndef _MSC_VER
    // 2DO: this test generates an assert error when using MSC compiler
    // need to track this down even though svec_setvec not currently being used
    ok     &= test_post<CppAD::local::sparse::svec_setvec>();
# endif
    //
    return ok;
}
