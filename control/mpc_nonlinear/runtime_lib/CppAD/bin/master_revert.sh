#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-21 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
if [ "$0" != 'bin/master_revert.sh' ]
then
    echo "bin/master_revert.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
branch=$(git branch | sed -n '/^[*]/p' | sed -e 's/[*] *//')
list=$(
    git diff master $branch | \
    sed -n -e '/^diff --git/p' | \
    sed -e 's|^diff --git a/||' -e 's| b/|@|'
)
for pair in $list
do
    master_file=$(echo $pair | sed -e 's|@.*||')
    branch_file=$(echo $pair | sed -e 's|[^@]*@||')
    if [ "$master_file" == "$branch_file" ]
    then
        echo "reverting $master_file"
        git show master:$master_file > $branch_file
    else
        echo 'skipping move of'
        echo "$master_file -> $branch_file"
    fi
done
echo 'master_revert.sh: OK'
exit 0
