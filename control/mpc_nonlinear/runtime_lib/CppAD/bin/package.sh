#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# Bradley M. Bell has given permission to use this script to generate
# a distribution of CppAD that has "GNU General Public License Version 3"
# in place of "Eclipse Public License Version 1.0.". Other's are free to
# keep or change this premission in their versions of the source code.
# -----------------------------------------------------------------------------
if [ "$0" != "bin/package.sh" ]
then
    echo "bin/package.sh: must be executed from its parent directory"
    exit 1
fi
include_doc='yes'
if [ "$1" != '' ]
then
    if [ "$1" == '--no_doc' ]
    then
        include_doc='no'
    else
        echo 'usage: bin/package.sh [--no_doc]'
        exit 1
    fi
fi
if [ "$include_doc" == 'yes' ]
then
    if ! git show-ref origin/gh-pages >& /dev/null
    then
        echo 'bin/package.sh: git cannot find origin/gh-pages branch'
        echo 'use the following command to fetch it:'
        echo '    git fetch origin gh-pages'
        exit 1
    fi
fi
echo_eval() {
     echo $*
     eval $*
}
# -----------------------------------------------------------------------------
src_dir=`pwd`
version=`sed -n -e '/^SET( *cppad_version *"[0-9.]*"/p' CMakeLists.txt | \
    sed -e 's|.*"\([^"]*\)".*|\1|'`
date=`echo $version | sed -e 's|\.[0-9]*$||'`
# -----------------------------------------------------------------------------
# doc
if [ -e 'doc' ]
then
    echo_eval rm -r doc
fi
cat << EOF > $src_dir/package.$$
/^commit/! b end
N
N
N
N
/version $date/! b end
s|\\nAuthor:.*||
s|commit *||
p
: end
EOF
# -----------------------------------------------------------------------------
if [ "$include_doc" == 'yes' ]
then
    #
    # use gh-pages if they exist for this version
    git_hash=`git log origin/gh-pages | sed -n -f $src_dir/package.$$ | head -1`
    if [ "$git_hash" != '' ]
    then
        echo 'create ./doc'
        mkdir doc
        list=`git ls-tree --name-only $git_hash:doc`
        for file in $list
        do
            git show $git_hash:doc/$file > doc/$file
        done
    elif which run_omhelp.sh > /dev/null
    then
        # omhelp is available, so build documentation for this version
        echo_eval run_omhelp.sh doc
    else
cat << EOF
Cannot find gh-pages documentation for this version: $version
and cannot find run_omhelp.sh on this system.

If you wish to skip the documentation for this system use
bin/package.sh --no_doc. Otherwise, use the following steps:

1. Use 'git log origin/gh-pages' to see which versions have docuimentation.

2. Pick a version number and search for 'Advance to' for that version number
in output of 'git log master' and note the <hash_code> for that commit.

3. Use 'git checkout <hash_code>' to check the source code for that version.

4. Use 'git show master:bin/package.sh > bin/package.sh' to get the most
recent version of bin/package.sh

5. Re-run  bin/package.sh"
EOF
        rm package.$$
        exit 1
    fi
fi
# -----------------------------------------------------------------------------
# change_list
cat << EOF > $src_dir/package.$$
/^.gitignore\$/d
/^authors\$/d
/^bin\\/colpack.sh\$/d
/^coin.png\$/d
/^compile\$/d
/^config.guess\$/d
/^config.sub\$/d
/^configure\$/d
/^depcomp\$/d
/^install-sh\$/d
/^missing\$/d
/^uw_copy_040507.html\$/d
EOF
change_list=`git ls-files | sed -f $src_dir/package.$$`
rm $src_dir/package.$$
# ----------------------------------------------------------------------------
# clean up old results
if [ ! -e 'build' ]
then
    echo_eval mkdir build
fi
echo_eval rm -rf build/cppad-*
echo_eval mkdir build/cppad-$version
# -----------------------------------------------------------------------------
# cppad-$version.tgz
git ls-files -z | xargs -0 tar -czf build/cppad-$version/tar.tgz
# -----------------------------------------------------------------------------
# cppad-$version.tgz
echo "create build/cppad-$version.tgz"
cd build/cppad-$version
tar -xzf tar.tgz
rm tar.tgz
if [ "$include_doc" == 'yes' ]
then
    cp -r ../../doc doc
fi
cd ..
tar --create cppad-$version --gzip --file=cppad-$version.tgz
rm -r cppad-$version
# ---------------------------------------------------------------------------
echo 'package.sh: OK'
exit 0
