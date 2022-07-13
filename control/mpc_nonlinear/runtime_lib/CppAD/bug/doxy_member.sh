#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-14 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Trying to figure out why ADFun::Forward appers twice where there is only
# one implementation.
#
# ------------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------
echo "$0"
name=`echo $0 | sed -e 's|.*/||' -e 's|\..*||'`
# -----------------------------------------------
for dir in build doxy_member
do
    if [ ! -e $dir ]
    then
        mkdir $dir
    fi
    cd $dir
done
# -------------------------------------------------------------------------
cat << EOF > $name.hpp

template<class T>
class my_class {
private:
    T value_;
public:
    void set_value(T value = 0);
    T  get_value(void);
};
EOF
cat << EOF > implement.hpp
/*!
\\file implement.hpp
Implementation of member functions
*/

/*!
Member function that sets the value.

\\param value [in]
New value.
*/
template<class T>
void my_class<T>::set_value(T value)
{   value_ = value; }

/*!
Member function that gets the value.

\\return
Current value.
*/
template<class T>
T my_class<T>::get_value(void)
{   return value_; }
EOF
cat << EOF > $name.cpp
# include <iostream>
# include "$name.hpp"
# include "implement.hpp"
int main(void)
{   my_class<int> x;
    x.set_value(2);
    std::cout << "x.value = " << x.get_value() << std::endl;
    return 0;
}
EOF
# -------------------------------------------------------------------------
# echo_eval doxygen -g doxyfile
cp ../../../doxyfile .
sed \
    -e 's|^\(INPUT *=\)|& .|' \
    -e 's|^\(FILE_PATTERNS *=\)|& *.hpp *.cpp|' \
    -i doxyfile
# -------------------------------------------------------------------------
echo_eval doxygen doxyfile
# -------------------------------------------------------------------------
echo_eval g++ $name.cpp -o name
echo_eval ./name
