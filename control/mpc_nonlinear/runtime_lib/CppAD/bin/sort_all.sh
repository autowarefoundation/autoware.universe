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
if [ $0 != "bin/sort_all.sh" ]
then
    echo "bin/sort_all.sh: must be executed from its parent directory"
    exit 1
fi
list=`git grep -l 'BEGIN_SORT_THIS_LINE_PLUS_' | sed \
    -e '/\/makefile.in$/d' \
    -e '\/^makefile.in$/d' \
    -e '/\/sort_all.sh$/d' `
#
for file in $list
do
    sort.sh $file
done
#
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
