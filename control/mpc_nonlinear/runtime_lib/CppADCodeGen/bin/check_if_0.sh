#! /bin/bash -e
# $Id: check_if_0.sh 2082 2011-08-31 17:50:58Z bradbell $
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-11 Bradley M. Bell
#
# CppAD is distributed under multiple licenses. This distribution is under
# the terms of the
#                     Common Public License Version 1.0.
#
# A copy of this license is included in the COPYING file of this distribution.
# Please visit http://www.coin-or.org/CppAD/ for information on other licenses.
# -----------------------------------------------------------------------------
if [ ! -e "bin/check_if_0.sh" ]
then
	echo "bin/check_if_0.sh: must be executed from its parent directory"
	exit 1
fi
# -----------------------------------------------------------------------------
# CppAD uses preprocessor '# if 0' comment blocks for temporary changes
# that will to be removed before testing for check in.
echo "Checking for '# if 0' comments blocks in source code"
echo "-------------------------------------------------------" 
ok="yes"
for ext in .cpp .hpp
do
	dir_list=`find . -name "*$ext" | \
		sed -e 's|^\./||' -e '/^work/d' -e 's|/[^/]*$||' | sort -u`  
	for dir in $dir_list 
	do
		list=`ls $dir/*$ext`
		for file in $list
		do
			if grep '^# *if *0 *$' $file > /dev/null
			then
				echo "$file has an '# if 0' comment block"
				ok="no"
			fi
		done
	done
done
echo "-------------------------------------------------------" 
if [ "$ok" = "no" ]
then
	echo "Error: nothing should be between the two dashed lines above"
	exit 1
else
	echo "Ok: nothing is between the two dashed lines above"
	exit 0
fi
