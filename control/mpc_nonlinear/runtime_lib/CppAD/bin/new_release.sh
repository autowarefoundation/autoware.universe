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
stable_version='20220000' # date at which this stable branch started
release='0'               # first release for each stable version is 0
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
if [ "$0" != 'bin/new_release.sh' ]
then
    echo "bin/new_release.sh: must be executed from its parent directory"
    exit 1
fi
#
branch=`git branch | grep '^\*'`
if [ "$branch" != '* master' ]
then
    echo 'new_release.sh: must start execution using master branch'
    exit 1
fi
#
# check that .coin-or/projDesc.xml and omh/cppad.omh are correct
key='stableVersionNumber'
sed -i .coin-or/projDesc.xml \
    -e "s|<$key>[0-9]*</$key>|<$key>$stable_version</$key>|"
#
key='releaseNumber'
sed -i .coin-or/projDesc.xml \
    -e "s|<$key>[0-9.]*</$key>|<$key>$stable_version.$release</$key>|"
#
# check stable version number
sed -i omh/cppad.omh \
    -e "/\/archive\//N" \
    -e "/\/archive\//s|[0-9]\{8\}\.[0-9]*|$stable_version.$release|g"
#
list=`git status -s`
if [ "$list" != '' ]
then
    git add --all
    echo "new_release.sh: 'git status -s' is not empty for master branch"
    echo "commit changes to master branch with the following command ?"
    echo "git commit -m 'master: bin/new_release.sh $stable_version.$release'"
    exit 1
fi
# -----------------------------------------------------------------------------
# Check if these reference tags already exist
#
tag=$stable_version.$release
if git tag --list | grep "$tag"
then
    echo "The reference tag $tag already exist"
    echo 'Use the following command to delete the old version ?'
    echo "    git tag -d $tag"
    echo "    git push --delete origin $tag"
    exit 1
fi
tag=$stable_version.doc
if git tag --list | grep "$tag"
then
    if [ "$release" == 0 ]
    then
        echo "The reference tag $tag already exist"
        echo 'Use the following command to delete the old version ?'
        echo "    git tag -d $tag"
        echo "    git push --delete origin $tag"
        exit 1
    fi
else
    if [ "$release" != 0 ]
    then
        echo "The reference tag $tag does not exist"
        echo 'But the release is not 0'
        exit 1
    fi
fi
# =============================================================================
# gh-pages
# =============================================================================
cat << EOF > new_release.$$
/^commit/! b end
N
N
N
N
/version $stable_version/! b end
s|\\nAuthor:.*||
s|commit *||
p
: end
EOF
# use gh-pages if they exist for this version
doc_hash=`git log origin/gh-pages | sed -n -f new_release.$$ | head -1`
if [ "$doc_hash" == '' ]
then
cat << EOF
Cannot find commit message for $stable_version in output of
git log origin/gh-pages. Use the following comands to fix this ?
    gh_pages.sh
    git commit -m 'update gh-pages to stable version $stable_version'
    git push
    git checkout master
EOF
    rm new_release.$$
    exit 1
fi
rm new_release.$$
# =============================================================================
# stable branch
# =============================================================================
stable_branch=stable/$stable_version
#
# checkout the stable branch
if ! git checkout $stable_branch
then
    echo "branch $stable_branch does not exist. Use following to create it ?"
    echo "git branch $stable_branch"
    exit 1
fi
#
# check version number
ok='yes'
check_one=`version.sh get`
if [ "$check_one" != "$stable_version.$release" ]
then
    ok='no'
fi
if ! version.sh check > /dev/null
then
    ok='no'
fi
if [ "$ok" != 'yes' ]
then
cat << EOF
bin/new_release.sh: version number is not correct in $stable_branch.
Currently in $stable_branch branch, use following to fix it ?
    git fetch
    version.sh set $stable_version.$release
    version.sh copy
    version.sh check
    Then check the chages to the $stable_branch branch and commit
EOF
    exit 1
fi
#
# local hash code
stable_local_hash=`git show-ref $stable_branch | \
    grep "refs/heads/$stable_branch" | \
    sed -e "s| *refs/heads/$stable_branch||"`
#
# remote hash code
stable_remote_hash=`git show-ref $stable_branch | \
    grep "refs/remotes/origin/$stable_branch" | \
    sed -e "s| *refs/remotes/origin/$stable_branch||"`
#
if [ "$stable_local_hash" == '' ] && [ "$stable_remote_hash" == '' ]
then
    echo "new_release.sh: $stable_branch does not exist"
    echo "Use following command to create it ?"
    echo "    git checkout -b $stable_branch master"
    echo "    version.sh set $stable_version.$release"
    echo '    version.sh copy'
    echo 'Then run tests. Then commit changes.'
    exit 1
fi
if [ "$stable_local_hash" == '' ] && [ "$stable_remote_hash" != '' ]
then
    echo "new_release.sh: local $stable_branch does not exist."
    echo "    git checkout -b $stable_branch origin/$stable_branch"
    exit 1
fi
#
if [ "$stable_remote_hash" == '' ]
then
    echo "new_release.sh: remote $stable_branch does not exist ?"
    echo "    git push origin $stable_branch"
    exit 1
fi
#
# check local == remote
if [ "$stable_local_hash" != "$stable_remote_hash" ]
then
    echo "new_release.sh: local and remote differ for $stable_branch"
    echo "local  $stable_local_hash"
    echo "remote $stable_remote_hash"
    echo 'try: git push'
    exit 1
fi
# =============================================================================
# master
# =============================================================================
# local hash code for master
master_local_hash=`git show-ref master | \
    grep "refs/heads/master" | \
    sed -e "s| *refs/heads/master||"`
#
# remote hash code
master_remote_hash=`git show-ref master | \
    grep "refs/remotes/origin/master" | \
    sed -e "s| *refs/remotes/origin/master||"`
#
if [ "$master_local_hash" != "$master_remote_hash" ]
then
    echo 'new_release.sh: local and remote for master differ'
    echo "local  $master_local_hash"
    echo "remote $master_remote_hash"
    echo 'try:   git checkout master'
    echo 'try:   git push'
    exit 1
fi
# -----------------------------------------------------------------------------
read -p 'All checks have passed. More testing or commit release [t/c] ?' \
    response
if [ "$response" != 'c' ]
then
    echo 'Exiting for more testing of stable branch'
    exit 1
fi
# -----------------------------------------------------------------------------
# tag the source code
#
echo_eval git tag -a -m \"created by bin/new_release.sh\" \
    $stable_version.$release $stable_remote_hash
#
echo_eval git push origin $stable_version.$release
#
# tag the documentation
#
if [ "$release" == '0' ]
then
    echo_eval git tag -a -m \"created by bin/new_release.sh\" \
        $stable_version.doc  $doc_hash
    #
    echo_eval git push origin $stable_version.doc
fi
# =============================================================================
# master branch
# =============================================================================
git checkout master
echo "$0: OK"
exit 0
