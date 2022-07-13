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
if [ ! -e "bin/check_tempfile.sh" ]
then
    echo "bin/check_tempfile.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
list=`ls | sed -n \
    -e '/^new.[0-9]*$/d' \
    -e '/^junk.[0-9]*$/d' \
    -e '/\/junk.[0-9]*$/d' \
    -e '/\.[0-9]*$/p'`
if [ "$list" != '' ]
then
    echo 'Use following command to remove temporary files:'
    cmd='rm '
    for file in $list
    do
        cmd="$cmd $file"
    done
    echo "    $cmd"
    echo 'check_tempfile.sh: Error'
    exit 1
fi
echo 'check_tempfile.sh: OK'
exit 0
