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
if [  "$0" != 'bin/deprecate_xam.sh' ]
then
    echo 'bin/deprecate_xam.sh: must be executed from its parent directory'
    exit 1
fi
if [ "$1" == '' ]
then
    echo 'bin/deprecate_xam.sh file'
    echo 'where file is an example file that is to be deprecated'
    exit 1
fi
old_file="$1"
ext=`echo $old_file | sed -e 's|.*\.||'`
if [ $ext != 'cpp' ]
then
    echo 'bin/depreate_xam.sh: file name does not end in .cpp'
    exit 1
fi
dir=`echo $old_file | sed -e 's|/.*||'`
if [ $dir != 'example' ]
then
    echo 'bin/depreate_xam.sh: file name does not start with example/'
    exit 1
fi
# -----------------------------------------------------------------------------
# move file
root_name=`echo $old_file | sed -e 's|.*/||' -e 's|\.cpp$||'`
new_dir="test_more/deprecated"
new_file="$new_dir/$root_name.cpp"
echo "$omh_name"
if [ -e $new_file ]
then
    git reset -- $new_file $old_file
    rm $new_file
    git checkout $old_file
fi
echo_eval git mv $old_file $new_file
# -----------------------------------------------------------------------------
# change old directory
old_dir=`echo $old_file | sed -e 's|/[^/]*$||'`
old_program=`echo $old_dir | sed -e 's|.*/||'`
git checkout $old_dir/CMakeLists.txt
sed -i $old_dir/CMakeLists.txt -e "/$root_name.cpp/d"
git checkout $old_dir/makefile.am
sed -i $old_dir/makefile.am -e "/$root_name.cpp/d"
git checkout $old_dir/$old_program.cpp
sed -i $old_dir/$old_program.cpp -e "/$root_name/d"
# -----------------------------------------------------------------------------
# change new directory
git checkout $new_dir/CMakeLists.txt
sed -i $new_dir/CMakeLists.txt \
    -e "s|deprecated.cpp|&\\n    $root_name.cpp|"
git checkout $new_dir/makefile.am
sed -i $new_dir/makefile.am \
    -e "s|deprecated.cpp.*|&\\n\\t$root_name.cpp \\\\|"
git checkout $new_dir/deprecated.cpp
sed -i $new_dir/deprecated.cpp \
    -e "s|bool old_mat_mul.*|&\\nextern bool $root_name(void);|" \
    -e "s|Run( old_mat_mul.*|&\\n    Run( $root_name, \"$root_name\" );|"
# -----------------------------------------------------------------------------
# file omhelp links
omh_name=`grep '$begin' $new_file | sed -e 's|$begin ||' -e 's|\$\$.*||'`
git checkout omh/example_list.omh
sed -i omh/example_list.omh -e "/$omh_name/d"
# -----------------------------------------------------------------------------
echo 'bin/deprecate_xam.sh: OK'
exit 0
