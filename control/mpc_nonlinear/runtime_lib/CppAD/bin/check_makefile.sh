#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#            GNU General Public License, Version 3.0.
# -----------------------------------------------------------------------------
if [ ! -e "bin/check_makefile.sh" ]
then
    echo "bin/check_makefile.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
echo 'Checking if any makfile.in has changed'
bin/autotools.sh automake >& /dev/null
list_in=`git ls-files '*/makefile.in'`
list_am=`git ls-files '*/makefile.am' | sed -e 's|\.am$|.in|'`
ok='yes'
for file in $list_in
do
    if ! echo $list_am | grep "$file" > /dev/null
    then
        echo "$file does name have a corresponding makefile.am"
        exit 1
    fi
    diff=`git diff -- $file`
    if [ "$diff" != '' ]
    then
        echo "git add $file"
        git add $file
        ok='no'
    fi
done
if [ "$ok" == 'no' ]
then
    echo '*/makefile.in has changed.'
    exit 1
fi
# -----------------------------------------------------------------------------
echo 'Checking *.cpp files and flags in makefile.am'
list=`git ls-files '*/makefile.am'`
ok='yes'
for file in $list
do
    dir=`echo $file | sed -e 's|/[^/]*$||'`
    cpp_list=`git ls-files $dir'/*.cpp' | sed -e "s|$dir/||" -e '/\//d'`
    for cpp in $cpp_list
    do
        if ! grep $cpp $file > /dev/null
        then
            echo "$cpp not in $file"
            ok='no'
        fi
    done
    if grep '\-DNDEBUG' $file > /dev/null
    then
        echo "-DNDEBUG flag appears in $file"
        ok="no"
    fi
    if grep '\-g' $file > /dev/null
    then
        echo "-g flag appears in in $file"
        ok="no"
    fi
done
if [ "$ok" == 'no' ]
then
    echo '*/makefile.am is missing some changes.'
    exit 1
fi
# -----------------------------------------------------------------------------
echo "Checking include files listed in include/makefile.am"
echo "-------------------------------------------------------"
git ls-files | sed -n \
    -e '/^include\/cppad\/.*\.hpp$/p'  \
    -e '/^include\/cppad\/.*\.hpp.in$/p' | \
    sed -e 's|^include/||' -e 's|\.in$||' > check_makefile.1.$$
# LC_ALL='C' is used by sort.sh for sorting order
export LC_ALL='C'
sort -u check_makefile.1.$$ > check_makefile.2.$$
#
sed < include/makefile.am -n \
    -e '/^nobase_myinclude_HEADERS *=/,/^# End nobase_myinclude_HEADERS/p' | \
    sed \
        -e '/nobase_myinclude_HEADERS/d' \
        -e 's/^\t//' \
        -e 's/ *\\$//' \
        -e 's/ *$//' \
        -e '/^$/d'  |
    sort > check_makefile.3.$$
#
if diff check_makefile.2.$$ check_makefile.3.$$
then
    ok="yes"
else
    ok="no"
fi
rm check_makefile.*.$$
echo "-------------------------------------------------------"
if [ "$ok" = "no" ]
then
    echo "Error: nothing should be between the two dashed lines above"
    exit 1
fi
echo "-------------------------------------------------------"
if [ "$ok" = "yes" ]
then
    echo "Ok: nothing is between the two dashed lines above"
    exit 0
else
    echo "Error: nothing should be between the two dashed lines above"
    exit 1
fi
