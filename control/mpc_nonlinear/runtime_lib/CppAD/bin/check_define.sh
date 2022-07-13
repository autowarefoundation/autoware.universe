#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ $0 != 'bin/check_define.sh' ]
then
    echo 'bin/check_define.sh: must be executed from its parent directory'
    exit 1
fi
# -----------------------------------------------------------------------------
echo 'Check '# define' versus '# undef' names and check for addon names'
echo '-----------------------------------------------------------------------'
file_list=`git ls-files '*.hpp' '*.in' |
    sed -n -e '/^include\/cppad\//p'`
add_on_list='CG PY TMB MIXED'
#
# preprocessor symbol that may or may not be defined by user
echo 'CPPAD_DEBUG_AND_RELEASE' > check_define.1
#
for file in $file_list
do
        include_guard=`echo $file | sed \
            -e 's|^include/||' \
            -e 's|\.in||' \
            -e 's|/|_|g' \
            -e 's|\.hpp|_hpp|' \
            | tr [a-z] [A-Z]
        `
        # define
        if [ ! -e $file.in ]
        then
            sed -n -e "/^# *define /p" $file | sed \
                -e "/^# *define *$include_guard/d" \
                -e '/^# define NOMINMAX/d' \
                -e "s/^# *define  *\([A-Za-z0-9_]*\).*/\1/" >> check_define.1
        fi
        # undef
        if [ ! -e $file.in ]
        then
            # note <cppad/local/utility/cppad_vector_itr.hpp> is special
            sed -n -e "/^# *undef /p" $file | sed \
                -e '/CPPAD_LOCAL_UTILITY_CPPAD_VECTOR_ITR_HPP/d' \
                -e "s/^# *undef  *\([A-Za-z0-9_]*\).*/\1/" >> check_define.2
        fi
        # add_on
        for add_on in $add_on_list
        do
            if grep "CPPAD_${add_on}_" $file
            then
                add_on_error='true'
            fi
        done
done
# sort lists
for file in check_define.1 check_define.2
do
    sort -u $file > check_define.3
    mv check_define.3 $file
done
if ! diff check_define.1 check_define.2
then
    echo 'check_define.sh: Error: defines and undefs do not match'
    rm check_define.1 check_define.2
    exit 1
fi
rm check_define.1 check_define.2
echo '-----------------------------------------------------------------------'
if [ "$add_on_error" == 'true' ]
then
    echo 'check_define.sh: Error: add_on preprocessor symbol found'
    exit 1
fi
echo 'check_define.sh: OK'
exit 0
