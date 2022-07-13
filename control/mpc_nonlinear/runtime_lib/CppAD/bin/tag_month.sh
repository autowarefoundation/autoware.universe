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
if [ $0 != "bin/tag_month.sh" ]
then
    echo "bin/tag_month.sh: must be executed from its parent directory"
    exit 1
fi
svn_repository="https://projects.coin-or.org/svn/CppAD"
# --------------------------------------------------------------------------
# make sure that master is currently checked out
git_branch=`git branch | sed -e '/^\*/! d' -e 's|^\* *||'`
if [ "$git_branch" != 'master' ]
then
    echo 'tag_month.sh: master is not currently checkout out'
    exit 1
fi
# --------------------------------------------------------------------------
# make sure that version is consistent and all changes are checked in
version.sh check
# --------------------------------------------------------------------------
# make sure repo is up to date
git_status=`git status -s`
if [ "$git_status" != '' ]
then
    echo 'tag_month.sh: master has changes that are not checked in'
    exit 1
fi
# --------------------------------------------------------------------------
# check that version corresponds to first of a month
dd=`version.sh get | sed -e 's|......\([0-9][0-9]\)|\1|' `
if [ "$dd" != '01' ]
then
    echo 'tag_month.sh: verison does not correspond to first day of a month'
    exit 1
fi
# --------------------------------------------------------------------------
# date of last change to svn repository
svn_date=`svn log $svn_repository/trunk --limit 1 | grep '^r[0-9]* *|' | \
    sed -e 's/^[^|]*|[^|]*| *\([0-9-]*\).*/\1/' -e 's|-||g'`
# --------------------------------------------------------------------------
# get and check hash codes
#
local_hash=`git show-ref master | sed -e '/\/origin\//d' -e 's| refs.*||'`
remote_hash=`git show-ref master | sed -e '/\/origin\//! d' -e 's| refs.*||'`
svn_hash=`svn log $svn_repository/trunk --limit 1 | \
    grep 'end *hash *code:' | sed -e 's|end *hash *code: *||'`
#
if [ "$local_hash" != "$remote_hash" ]
then
    echo "tag_month.sh: master changes haven't been pushed to git repository"
    echo "local_hash  = $local_hash"
    echo "remote_hash = $remote_hash"
    exit 1
fi
if [ "$remote_hash" != "$svn_hash" ]
then
    echo "tag_month.sh: master changes haven't been pushed to svn repository"
    echo "remote_hash = $remote_hash"
    echo "svn_hash    = $svn_hash"
    exit 1
fi
# -----------------------------------------------------------------------------
# If this version has already been tagged, delete the tag
version=`version.sh get`
if git tag --list | grep "$version"
then
    read -p "Delete preious tag for version $version [y/n] ?" response
    if [ "$response" != 'y' ]
    then
        echo 'tag_month.sh: aborting because tag already exists'
        exit 1
    fi
    git tag -d $version
    git push --delete origin $version
fi
#
echo "git tag -a \\"
echo "-m \"Last changes copied to $svn_repository/trunk on $svn_date\" \\"
echo "$version $svn_hash"
git tag -a \
    -m "Last changes copied to $svn_repository/trunk on $svn_date" \
    $version $svn_hash
#
echo "git push origin $version"
git push origin $version
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
