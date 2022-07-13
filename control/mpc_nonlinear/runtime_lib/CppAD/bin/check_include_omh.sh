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
if [ ! -e "bin/check_include_omh.sh" ]
then
    echo "bin/check_include_omh.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
count=`git ls-files include/cppad | grep '/omh/.*\.hpp' | wc -l`
if [ "$count" != '0' ]
then
    git ls-files include/cppad | grep '/omh/.*\.hpp'
    echo 'Cannot put *.hpp files below omh in include directory'
    echo 'because install of include directory will exclude them.'
    exit 1
fi
echo 'check_include_omh: OK'
exit 0
