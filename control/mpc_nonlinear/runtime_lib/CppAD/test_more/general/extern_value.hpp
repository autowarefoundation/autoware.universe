# ifndef CPPAD_TEST_MORE_GENERAL_EXTERN_VALUE_HPP
# define CPPAD_TEST_MORE_GENERAL_EXTERN_VALUE_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-17 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

template <class Type>
class extern_value  {
private:
    Type value_;
public:
    extern_value(Type value);
    void set(Type value);
    Type get(void);
};

# endif
