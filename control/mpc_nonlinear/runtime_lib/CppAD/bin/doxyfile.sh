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
if [ ! -e "bin/doxyfile.sh" ]
then
    echo "bin/doxyfile.sh: must be executed from its parent directory"
    exit 1
fi
# -----------------------------------------------------------------------------
if [ "$3" == "" ]
then
    echo "usage: bin/doxyfile.sh version error_file output_directory"
    echo "creates the doxygen configuration file ./doxyfile"
    exit 1
fi
version="$1"
error_file="$2"
output_directory="$3"
# -----------------------------------------------------------------------------
# convert multi-line assignments to single line assignments.
echo "doxygen -g doxyfile > /dev/null"
doxygen -g doxyfile > /dev/null
cat << EOF > doxyfile.$$
/^[A-Z_]* *=.*\\\\$/! b end
: loop
N
/\\\\$/b loop
s|\\\\\\n| |g
s|  *| |g
#
:end
EOF
sed -i doxyfile -f doxyfile.$$
# -----------------------------------------------------------------------------
include_directory_list=`find include/cppad -type d | tr '\n' ' ' `
# -----------------------------------------------------------------------------
# PREDEFINED:see http://www.stack.nl/~dimitri/doxygen/manual/preprocessing.html
# 2DO: change EXTRACT_ALL to NO so get warnings for undocumented functions.
echo "create doxyfile.$$"
cat << EOF > doxyfile.$$
ALWAYS_DETAILED_SEC     = YES
BUILTIN_STL_SUPPORT     = YES
ENABLE_PREPROCESSING    = YES
MACRO_EXPANSION         = YES
EXTRACT_ALL             = YES
EXTRACT_LOCAL_CLASSES   = YES
EXTRACT_PRIVATE         = YES
EXTRACT_STATIC          = YES
EXTRACT_ANON_NSPACES    = YES
FILE_PATTERNS           =  *.hpp  *.cpp
FULL_PATH_NAMES         = NO
GENERATE_HTML           = YES
GENERATE_LATEX          = NO
GENERATE_TREEVIEW       = YES
INHERIT_DOCS            = NO
INLINE_INHERITED_MEMB   = YES
INPUT                   = $include_directory_list
LATEX_BATCHMODE         = YES
MACRO_EXPANSION         = YES
MULTILINE_CPP_IS_BRIEF  = YES
OUTPUT_DIRECTORY        = $output_directory
PREDEFINED              = "__cplusplus=201103" "CPPAD_USE_CPLUSPLUS_2011=1"
PROJECT_NAME            = "CppAD: A C++ Algorithmic Differentiation Package"
PROJECT_NUMBER          = $version
QT_AUTOBRIEF            = YES
REFERENCED_BY_RELATION  = YES
REFERENCES_LINK_SOURCE  = NO
SEPARATE_MEMBER_PAGES   = YES
SHOW_DIRECTORIES        = YES
SHOW_INCLUDE_FILES      = NO
SORT_GROUP_NAMES        = YES
SORT_MEMBER_DOCS        = YES
SORT_BRIEF_DOCS         = YES
SOURCE_BROWSER          = YES
STRIP_CODE_COMMENTS     = NO
SUBGROUPING             = NO
TAB_SIZE                = 5
VERBATIM_HEADERS        = NO
WARN_LOGFILE            = $error_file
WARN_NO_PARAMDOC        = YES
EOF
sed \
    -e 's/^/s|^\\(/' \
    -e 's/ *=/ *=\\).*|\\1/' \
    -e 's/$/|/' \
    -i doxyfile.$$
#
#
echo "sed -f doxyfile.$$ -i doxyfile"
sed -f doxyfile.$$ -i doxyfile
#
echo "rm doxyfile.$$"
rm doxyfile.$$
# ----------------------------------------------------------------------------
echo "$0: OK"
exit 0
