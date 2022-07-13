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
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [  "$0" != 'bin/check_automake.sh' ]
then
    echo "bin/check_automake: must be executed from its parent directory"
    exit 1
fi
dir=$(ls /usr/share | grep automake)
dir="/usr/share/$dir"
ok='yes'
for name in $(ls)
do
    if [ -e $dir/$name ]
    then
        if [ -L $name ]
        then
            echo "$name should be a file not a link"
            exit 1
        fi
        if [ $name != 'COPYING' ]
        then
            if ! diff $dir/$name $name > /dev/null
            then
                ok='no'
                echo "cp $dir/$name $name"
            fi
        fi
    fi
done
if [ "$ok" == 'no' ]
then
    echo 'check_automake.sh: commands above will update automake files'
    exit 1
fi
echo 'check_sutomake.sh: OK'
exit 0
