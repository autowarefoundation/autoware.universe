#! /bin/bash -e
# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-20 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
# build script for use with autotools install
# -----------------------------------------------------------------------------
# prefix directories for the corresponding packages
#
# This test script no longer works with the current version of ADOLC
# ADOLC_DIR=$HOME/prefix/adolc
#
BOOST_DIR=/usr
CPPAD_DIR=$HOME/prefix/cppad
EIGEN_DIR=$HOME/prefix/eigen
FADBAD_DIR=$HOME/prefix/fadbad
IPOPT_DIR=$HOME/prefix/ipopt
# -----------------------------------------------------------------------------
# bash function that echos and executes a command
echo_eval() {
    echo $*
    eval $*
}
# -----------------------------------------------------------------------------
if [ $0 != "bin/autotools.sh" ]
then
    echo "bin/autotools.sh: must be executed in the directory that contians it"
    exit 1
fi
if [ "$1" != 'automake' ] && [ "$1" !=  'configure' ] && [ "$1" != 'test' ]
then
     echo "$1 is not a valid option"
cat << EOF
usage: bin/autotools.sh option

where option is one of the following:
automake: run the tools required by autoconf and automake.
configure:run the configure script in the build directory.
test:     build and run the tests using the autotools.
EOF
    exit 1
fi
option="$1"
# -----------------------------------------------------------------------------
if [ ! -e build ]
then
    echo_eval mkdir build
fi
log_dir=`pwd`
log_file="$option.log"
version=$( grep '^AC_INIT(' configure.ac | \
    sed -e 's|AC_INIT([^,]*, *\[ *\([0-9.]* *\)\].*|\1|')
#
# Files are created by the configure command and copied to the source tree
configure_file_list="
    include/cppad/configure.hpp
"
# -----------------------------------------------------------------------------
if [ "$option" = "automake" ]
then
    # check that autoconf and automake output are in original version
    makefile_in=`sed configure.ac \
    -n \
    -e '/END AC_CONFIG_FILES/,$d' \
    -e '1,/AC_CONFIG_FILES/d' \
    -e 's|/makefile$|&.in|' \
    -e '/\/makefile.in/p'`
    auto_output="
        depcomp
        install-sh
        missing
        configure
        config.guess
        config.sub
        $makefile_in
    "
    missing=""
    for name in $auto_output
    do
        if [ ! -e $name ]
        then
            if [ "$missing" != "" ]
            then
                missing="$missing, $name"
            else
                missing="$name"
            fi
        else
            # force remake of files
            rm "$name"
        fi
    done
    if [ "$missing" != "" ]
    then
        echo "The following files:"
        echo "    $missing"
        echo "are not in subversion repository."
        echo "Check them in when this command is done completes."
    fi
    #
    echo_eval aclocal
    #
    echo "skipping libtoolize"
    # echo "libtoolize -c -f -i"
    # if ! libtoolize -c -f -i
    # then
    #   exit 1
    # fi
    #
    echo_eval autoconf
    #
    echo_eval automake --add-missing
    #
    link_list="missing install-sh depcomp config.sub config.guess"
    for name in $link_list
    do
        if [ -h "$name" ]
        then
            echo "Converting $name from a link to a regular file"
            #
            echo_eval cp $name $name.$$
            #
            echo_eval mv $name.$$ $name
        fi
    done
    #
    echo "OK: bin/autotools.sh automake"
    exit 0
fi
# -----------------------------------------------------------------------------
# configure
if [ "$option" == "configure" ]
then
    #
    echo_eval cd build
    #
    dir_list="
        --prefix=$CPPAD_DIR
    "
    testvector='cppadvector'
    if [ -e $BOOST_DIR/include/boost ]
    then
        dir_list="$dir_list BOOST_DIR=$BOOST_DIR"
        testvector='boostvector'
    fi
    if [ -e $EIGEN_DIR/include/Eigen ]
    then
        dir_list="$dir_list EIGEN_DIR=$EIGEN_DIR"
        testvector='eigenvector'
    fi
    if [ -e $FADBAD_DIR/include/FADBAD++ ]
    then
        dir_list="$dir_list FADBAD_DIR=$FADBAD_DIR"
    fi
    if [ -e $IPOPT_DIR/include/coin-or/IpIpoptApplication.hpp ]
    then
        dir_list="$dir_list IPOPT_DIR=$IPOPT_DIR"
    fi
    cxx_flags="-Wall -ansi -pedantic-errors -std=c++11 -Wshadow -DNDEBUG -O2"
    cxx_flags="$cxx_flags -isystem $EIGEN_DIR/include"
cat << EOF
../configure > $log_file \\
$dir_list \\
CXX_FLAGS=\"$cxx_flags\" \\
OPENMP_FLAGS=-fopenmp \\
--with-$testvector
EOF
    #
    ../configure > $log_dir/$log_file \
        $dir_list \
        CXX_FLAGS="$cxx_flags" \
        OPENMP_FLAGS=-fopenmp \
        --with-$testvector
    #
    for file in $configure_file_list
    do
        echo_eval cp $file ../$file
    done
    #
    echo "OK: bin/autotools.sh configure"
    exit 0
fi
# -----------------------------------------------------------------------------
if [ "$option" = "test" ]
then
    echo "date >> $log_file"
    date >> $log_dir/$log_file
    # -----------------------------------------------------------------------
    # build/cppad-$version.tgz
    bin/package.sh --no_doc
    # --------------------------------------------------------------
    echo_eval cd build
    #
    # create distribution directory
    echo_eval tar -xzf cppad-$version.tgz
    #
    # change into distribution directory
    echo_eval cd cppad-$version
    #
    echo "bin/autotools.sh configure >> $log_file"
    bin/autotools.sh configure >> $log_dir/$log_file
    #
    # ----------------------------------------------------------------------
    echo_eval cd build
    make_log="$log_dir/make_test.log"
    #
    # build and run all the tests
    echo "make test  >& make_test.log"
    make test >& $make_log
    #
    if grep ': *warning: .*tmpnam.*is dangerous' $make_log > /dev/null
    then
        grep ': *warning: .*tmpnam.*is dangerous' $make_log | head -1
    fi
    echo "cat make_test.log >> $log_file"
    cat $make_log >> $log_dir/$log_file
    #
    sed -i $make_log -e '/: *warning: .*tmpnam.*is dangerous/d'
    if grep ': *warning:' $make_log
    then
        echo "There are warnings in make_test.log"
        exit 1
    fi
    echo "rm make_test.log"
    rm $make_log
    # --------------------------------------------------------------------
    echo "date >> $log_file"
    date >> $log_dir/$log_file
    #
    echo "OK: bin/autotools.sh test"
    exit 0
fi
