# -----------------------------------------------------------------------------
# CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-22 Bradley M. Bell
#
# CppAD is distributed under the terms of the
#              Eclipse Public License Version 2.0.
#
# This Source Code may also be made available under the following
# Secondary License when the conditions for such availability set forth
# in the Eclipse Public License, Version 2.0 are satisfied:
#       GNU General Public License, Version 2.0 or later.
# -----------------------------------------------------------------------------
dnl Process this file with autoconf to produce a configure script.
dnl   package   version              bug-report
AC_INIT([cppad], [20220000.4], [cppad@list.coin-or.org])
AM_SILENT_RULES([no])

dnl By defalut disable maintainer mode when running configure;
dnl --enable-maintainer-mode will enable it.
dnl This avoids the autoconf and automake tools being launched by make
AM_MAINTAINER_MODE

dnl Set the default install prefix to the installers home directory
AC_PREFIX_DEFAULT(${HOME})

dnl Run any macros required for proper operation of generated makefiles
dnl Note this must come before AC_PROG_CC; see
dnl http://lists.gnu.org/archive/html/automake/2010-08/msg00113.html
dnl
dnl nostdinc: Do not define standard include directories in generated makefiles
dnl subdir-objects: create objects in sub-directories, same structure source
dnl foreign: not GNU standard; no NEWS, AUTHORS, ...
AM_INIT_AUTOMAKE([nostdinc subdir-objects -Wall foreign])

dnl Have configure check source directory for the following file:
dnl include/cppad/cppad.hpp
AC_CONFIG_SRCDIR(include/cppad/cppad.hpp)

# check if we have ar program and abort if it is not found
AM_PROG_AR

dnl -----------------------------------------------------------------------
dnl The case where enable_msvc != no is defined and supported by Ted Ralphs
AC_MSG_CHECKING([--enable-msvc])
AC_ARG_ENABLE([msvc],
    [AC_HELP_STRING([--enable-msvc],
    [Prefer (i)cl/ifort/link over GNU on MinGW/Cygwin.])],
    [enable_msvc=$enableval],
    [enable_msvc=no]
)
AC_MSG_RESULT([$enable_msvc])

dnl the prefix directory corresponding to CppAD -------------------------
dnl The following seems to work but NONE is not in autoconf specifications
AC_MSG_CHECKING([--prefix])
if test "$prefix" = "NONE" ; then
    AC_MSG_RESULT([$HOME])
else
    AC_MSG_RESULT([$prefix])
fi

dnl Are we using standard vectors ----------------------------------------
AC_MSG_CHECKING([--with-stdvector])
AC_ARG_WITH(stdvector,
    AC_HELP_STRING([--with-stdvector], [default is no]),
    [stdvector="yes"],
    [stdvector="no"]
)
if test "$stdvector" = "yes" ; then
    AC_SUBST(cppad_stdvector, 1)
else
    AC_SUBST(cppad_stdvector, 0)
fi
AC_MSG_RESULT([$stdvector])

dnl Are we using boost vectors ----------------------------------------
AC_MSG_CHECKING([--with-boostvector])
AC_ARG_WITH(boostvector,
    AC_HELP_STRING([--with-boostvector], [default is no]),
    [boostvector="yes"],
    [boostvector="no"]
)
AC_MSG_RESULT([$boostvector])
if test "$boostvector" = "yes" ; then
    if test "$stdvector" != "no" ; then
        AC_MSG_ERROR([cannot --with both stdvector and boostvector])
    fi
    AC_SUBST(cppad_boostvector, 1)
else
    AC_SUBST(cppad_boostvector, 0)
fi

dnl Are we using eigen vectors ----------------------------------------
AC_MSG_CHECKING([--with-eigenvector])
AC_ARG_WITH(eigenvector,
    AC_HELP_STRING([--with-eigenvector], [default is no]),
    [eigenvector="yes"],
    [eigenvector="no"]
)
AC_MSG_RESULT([$eigenvector])
if test "$eigenvector" = "yes" ; then
    if test "$stdvector" != "no" ; then
        AC_MSG_ERROR([cannot --with both stdvector and eigenvector])
    fi
    if test "$boostvector" != "no" ; then
        AC_MSG_ERROR([cannot --with both boostvector and eigenvector])
    fi
    AC_SUBST(cppad_eigenvector, 1)
else
    AC_SUBST(cppad_eigenvector, 0)
fi

dnl The postfix directiory corresponding to CppAD ------------------------
AC_MSG_CHECKING([POSTFIX_DIR])
AC_ARG_VAR(POSTFIX_DIR,
    [postfix directory for CppAD installation]
)
AM_CONDITIONAL(CppAD_POSTFIX, test "$POSTFIX_DIR" != "")
if test "$POSTFIX_DIR" != "" ; then
    AC_MSG_RESULT([$POSTFIX_DIR])
else
    AC_MSG_RESULT([no])
fi

dnl The prefix corresponding to the installation of Adolc ---------------
AC_MSG_CHECKING([ADOLC_DIR])
AC_ARG_VAR(ADOLC_DIR,
    [value of configure prefix directory during Adolc install]
)
AM_CONDITIONAL(CppAD_ADOLC, test "$ADOLC_DIR" != "")
if test "$ADOLC_DIR" != "" ; then
    AC_MSG_RESULT([$ADOLC_DIR])
    AC_SUBST(cppad_has_adolc, 1)
else
    AC_MSG_RESULT([no])
    AC_SUBST(cppad_has_adolc, 0)
fi

dnl The prefix corresponding to the installation of Eigen ---------------
AC_MSG_CHECKING([EIGEN_DIR])
AC_ARG_VAR(EIGEN_DIR,
    [value of configure prefix directory during Eigen install]
)
AM_CONDITIONAL(CppAD_EIGEN, test "$EIGEN_DIR" != "")
if test "$EIGEN_DIR" != "" ; then
    AC_MSG_RESULT([$EIGEN_DIR])
    AC_SUBST(EIGEN_INCLUDE, [-I$EIGEN_DIR/include])
    AC_SUBST(cppad_has_eigen, 1)
else
    AC_MSG_RESULT([no])
    AC_SUBST(EIGEN_INCLUDE, [])
    AC_SUBST(cppad_has_eigen, 0)
fi

dnl The directory corresponding to the installation of FADBAD -----------
AC_MSG_CHECKING([FADBAD_DIR])
AC_ARG_VAR(FADBAD_DIR,
    [parent directory directly above FADBAD++ directory]
)
AM_CONDITIONAL(CppAD_FADBAD, test "$FADBAD_DIR" != "")
if test "$FADBAD_DIR" != "" ; then
    AC_MSG_RESULT([$FADBAD_DIR])
    AC_SUBST(cppad_has_fadbad, 1)
else
    AC_MSG_RESULT([no])
    AC_SUBST(cppad_has_fadbad, 0)
fi

dnl The directory corresponding to the installation of SACADO -----------
AC_MSG_CHECKING([SACADO_DIR])
AC_ARG_VAR(SACADO_DIR,
    [value of the configure prefix directory during Sacado install]
)
AM_CONDITIONAL(CppAD_SACADO, test "$SACADO_DIR" != "")
if test "$SACADO_DIR" != "" ; then
    AC_MSG_ERROR([CppAD autotools build no longer support sacado, use cmake])
else
    AC_MSG_RESULT([no])
    AC_SUBST(cppad_has_sacado, 0)
fi

dnl The directory corresponding to the installation of IPOPT -----------
AC_MSG_CHECKING([IPOPT_DIR])
AC_ARG_VAR(IPOPT_DIR,
    [value of the configure prefix directory during Ipopt install]
)
AM_CONDITIONAL(CppAD_IPOPT, test "$IPOPT_DIR" != "")
if test "$IPOPT_DIR" != "" ; then
    AC_MSG_RESULT([$IPOPT_DIR])
    AC_SUBST(cppad_has_ipopt, 1)
else
    AC_MSG_RESULT([no])
    AC_SUBST(cppad_has_ipopt, 0)
fi

dnl The directory corresponding to the installation of BOOST ------------
AC_MSG_CHECKING([BOOST_DIR])
AC_ARG_VAR(BOOST_DIR,
    [parent directory directly above boost directory]
)
AM_CONDITIONAL(CppAD_BOOST_DIR, test "$BOOST_DIR" != "")
if test "$BOOST_DIR" != "" ; then
    AC_MSG_RESULT([$BOOST_DIR])
    AC_SUBST(BOOST_INCLUDE, [-I$BOOST_DIR/include])
    AC_SUBST(cppad_has_boost, 1)
else
    AC_MSG_RESULT([no])
    AC_SUBST(BOOST_INCLUDE,[])
    AC_SUBST(cppad_has_boost, 0)
fi

dnl The C++ compiler flags CXX_FLAGS -------------------------------------
dnl see AC_SUBST(CXXFLAGS, "") next to AC_PROG_CXX
#
AC_MSG_CHECKING([CXX_FLAGS])
AC_ARG_VAR(CXX_FLAGS,
    [ CXXFLAGS is ignored, use CXX_FLAGS for compiler flags]
)
# cxx_flags for fadbad and eigen
if test "$CXX_FLAGS" != "" ; then
    AC_MSG_RESULT([$CXX_FLAGS])
else
    AC_MSG_RESULT([no])
fi

dnl The compiler OpenMP flags OPENMP_FLAGS ----------------------------------
AC_MSG_CHECKING([OPENMP_FLAGS])
AC_ARG_VAR(OPENMP_FLAGS,
    [ flags that instruct compiler to interpret OpenMP directrives ]
)
if test "$OPENMP_FLAGS" != "" ; then
    AC_MSG_RESULT([$OPENMP_FLAGS])
else
    AC_MSG_RESULT([no])
fi
AM_CONDITIONAL(CppAD_OPENMP, test "$OPENMP_FLAGS" != "")

dnl The compiler OpenMP flags MAX_NUM_THREADS -------------------------------
AC_MSG_CHECKING([MAX_NUM_THREADS])
AC_ARG_VAR(MAX_NUM_THREADS,
    [ maximum number of threads that can be used with CppAD ]
)
if test "$MAX_NUM_THREADS" != "" ; then
    AC_MSG_RESULT([$MAX_NUM_THREADS])
    if test "$MAX_NUM_THREADS" -lt "4" ; then
        AC_MSG_ERROR(
            [ MAX_NUM_THREADS=$MAX_NUM_THREADS is less than 4 ]
        )
    fi
    max_num_threads="$MAX_NUM_THREADS"
else
    max_num_threads="48"
    AC_MSG_RESULT([no, using default value $max_num_threads])
fi
AC_SUBST(cppad_max_num_threads, [$max_num_threads])

dnl type used for addresses in AD tapes [ default = unsigned int ] -----------
AC_MSG_CHECKING([TAPE_ADDR_TYPE])
AC_ARG_VAR(TAPE_ADDR_TYPE, [type used for addresses in AD tapes])
if test "$TAPE_ADDR_TYPE" == "" ; then
    addr_t="unsigned int"
else
    addr_t="$TAPE_ADDR_TYPE"
fi
AC_SUBST(cppad_tape_addr_type, [$addr_t])
AC_MSG_RESULT([$addr_t])
ok="no"
if test "$addr_t" == "size_t" ; then
    ok="yes"
fi
if test "$addr_t" == "int" ; then
    AC_MSG_WARN(
        [TAPE_ADDR_TYPE=int is for CppAD developers (not users)]
    )
    ok="yes"
fi
if test "$addr_t" == "unsigned int" ; then
    ok="yes"
fi
if test "$addr_t" == "unsigned short int" ; then
    ok="yes"
fi
if test "$ok" == "no" ; then
    AC_MSG_ERROR(
        [$addr_t is not a valid choice for TAPE_ADDR_TYPE]
    )
fi

dnl type used for tape identifier [ default = unsigned int ] -----------
AC_MSG_CHECKING([TAPE_ID_TYPE])
AC_ARG_VAR(TAPE_ID_TYPE, [type used for addresses in AD tapes])
if test "$TAPE_ID_TYPE" == "" ; then
    tape_id_t="unsigned int"
else
    tape_id_t="$TAPE_ID_TYPE"
fi
AC_SUBST(cppad_tape_id_type, [$tape_id_t])
AC_MSG_RESULT([$tape_id_t])
ok="no"
if test "$tape_id_t" == "size_t" ; then
    ok="yes"
fi
if test "$tape_id_t" == "int" ; then
    AC_MSG_WARN(
        [TAPE_ID_TYPE=int is for CppAD developers (not users)]
    )
    ok="yes"
fi
if test "$tape_id_t" == "unsigned int" ; then
    ok="yes"
fi
if test "$tape_id_t" == "unsigned short int" ; then
    ok="yes"
fi
if test "$ok" == "no" ; then
    AC_MSG_ERROR(
        [$tape_id_t is not a valid choice for TAPE_ID_TYPE]
    )
fi
AC_MSG_NOTICE([********** End Checking Command Line arguments ***************])

dnl --------------------------------------------------------------------------
if test "$enable_msvc" == "no"
then
    cxx_list="g++ clang++ CC pgCC icpc gpp cxx cc++"
    cc_list="gcc clang cc pgcc icc"
    fc_list="gfortran ifort g95 fort77 f77 g77 pgf90 pgf77 ifc frt af77"
else
    dnl This case defined and supported by Ted Ralphs
    cxx_list="icl cl g++"
    cc_list="icl cl gcc"
    fc_list="ifort gfortran"
fi
dnl --------------------------------------------------------------------------

dnl AC_PROG_CXX([compiler-search-list]) outputs CXX as C++ compiler to use
AC_PROG_CXX([$cxx_list])
dnl erase any setting of CXXFLAGS, see CXX_FLAGS documentation
AC_SUBST(CXXFLAGS, "")

dnl AC_PROG_CC([compiler-search-list]) outputs CC as C++ compiler to use
AC_PROG_CC([$cc_list])
dnl erase any setting of CFLAGS
AC_SUBST(CFLAGS, "")

dnl AC_PROG_FC ([compiler-search-list], [dialect])
AC_PROG_FC([$fc_list])
dnl not compiling any fortran so no need to touch flags

dnl Check for pkgconfig
AC_CHECK_PROG(have_pkg_config, [pkg-config], [yes], [no])
AM_CONDITIONAL(CppAD_PKG_CONFIG, test "$have_pkg_config" = "yes")
dnl --------------------------------------------------------------------------
dnl If the compiler does not support -c -o, wrap it with the compile script.
AC_PROG_CXX_C_O
if test $ac_cv_prog_cxx_c_o = no ; then
   CXX="$am_aux_dir/compile $CXX"
fi
dnl -------------------------------------------------------------------------
dnl Setting that cmake finds but autotools not kept up to date on
AC_SUBST(cppad_cxx_flags, "")
AC_SUBST(cppad_has_colpack, 0)
AC_SUBST(cppad_has_mkstemp, 0)
AC_SUBST(cppad_has_tmpnam_s, 0)
AC_SUBST(compiler_has_conversion_warn, 0)

dnl -------------------------------------------------------------------------
dnl Determine which definition we are using for CPPAD_TESTVECTOR
if test "$stdvector$boostvector$eigenvector" = "nonono" ; then
    AC_SUBST(cppad_cppadvector, 1)
else
    AC_SUBST(cppad_cppadvector, 0)
fi

dnl the gettimeofday check automatically generates a yes / no configure msg
AC_CHECK_FUNC(
    gettimeofday,
    [gettimeofday="yes"],
    [gettimeofday="no"]
)
if test "$gettimeofday" = "yes" ; then
    AC_SUBST(cppad_has_gettimeofday, 1)
else
    AC_SUBST(cppad_has_gettimeofday, 0)
fi

dnl Use LIBTOOL for cppad_ipopt ?
dnl AC_PROG_LIBTOOL

dnl Determine if ranlib is present and where it is if present (set RANLIB)
AC_PROG_RANLIB

AC_MSG_CHECKING([whether using Microsoft C++ compiler])
AM_CONDITIONAL(CppAD_MS_COMPILER, test "$CXX" == "cl" || test "$CXX" == "icl")
if test "$CXX" == "cl" || test "$CXX" == "icl"; then
    AC_MSG_RESULT([yes])
else
    AC_MSG_RESULT([no])
fi

dnl AC_PROG_CP outputs CPP as command to run C preprocessor
AC_PROG_CPP

dnl Do nothing if complier supports inline, otherwise try to find a subsitute
dnl definition or, if cannot, define inline as empty
AC_C_INLINE

dnl determine FCLIBS, flags necessary for linking C++ with modern fortran
AC_FC_LIBRARY_LDFLAGS

dnl absolute path corresponding to top_builddig and top_srcdir in makefile.am
dnl (used by cppad-uninstalled.pc.in).
abs_top_builddir=`pwd`
AC_SUBST(ABS_TOP_BUILDDIR, "$abs_top_builddir")
abs_top_srcdir=`cd $srcdir ; pwd`
AC_SUBST(ABS_TOP_SRCDIR, "$abs_top_srcdir")
dnl --------------------------------------------------------------------------
dnl Check for C libraries

dnl check for pthread library
AC_CHECK_LIB(
    [pthread],
    [pthread_barrier_wait],
    [pthread="yes"],
    [pthread="no"]
)
AM_CONDITIONAL(CppAD_PTHREAD, test "$pthread" = "yes")
if test "$pthread" == "yes" ; then
    AC_SUBST(PTHREAD_LIB,"-lpthread")
else
    AC_SUBST(PTHREAD_LIB,"")
fi

dnl check for dl library
AC_SUBST(DL_LIB, "")
AC_CHECK_LIB(
    [dl],
    [dlopen],
    [DL_LIB="-ldl"],
    [DL_LIB=""]
)
dnl --------------------------------------------------------------------------
dnl check for boost thread library
AC_LANG_PUSH([C++])
LDFLAGS_save=$LDFLAGS
CXXFLAGS_save=$CXXFLAGS
boost_thread_found='no'
if test "$BOOST_DIR" != '' ; then
    CXXFLAGS="-I$BOOST_DIR/include $CXXFLAGS"
fi
if test "$BOOST_DIR" == '' && test "$boost_thread_found" == 'no' ; then
    bthread_lib='-lboost_thread-mt'
    LDFLAGS="$LDFLAGS_save $bthread_lib"
    AC_LINK_IFELSE(
        [AC_LANG_PROGRAM(
            [# include <boost/thread.hpp>],
            [boost::barrier wait(1);]
        )],
        [boost_thread_found='yes'],
        [boost_thread_found='no']
    )
fi
if test "$BOOST_DIR" != '' && test "$boost_thread_found" == 'no' ; then
    bthread_lib="$BOOST_DIR/lib/libboost_thread-mt.so"
    LDFLAGS="$LDFLAGS_save $bthread_lib"
    AC_LINK_IFELSE(
        [AC_LANG_PROGRAM(
            [# include <boost/thread.hpp>],
            [boost::barrier wait(1);]
        )],
        [boost_thread_found='yes'],
        [boost_thread_found='no']
    )
fi
if test "$BOOST_DIR" != '' && test "$boost_thread_found" == 'no' ; then
    bthread_lib="$BOOST_DIR/lib/libboost_thread-mt.a"
    LDFLAGS="$LDFLAGS_save $bthread_lib"
    AC_LINK_IFELSE(
        [AC_LANG_PROGRAM(
            [# include <boost/thread.hpp>],
            [boost::barrier wait(1);]
        )],
        [boost_thread_found='yes'],
        [boost_thread_found='no']
    )
fi
if test "$BOOST_DIR" == '' && test "$boost_thread_found" == 'no' ; then
    bthread_lib='-lboost_thread'
    LDFLAGS="$LDFLAGS_save $bthread_lib"
    AC_LINK_IFELSE(
        [AC_LANG_PROGRAM(
            [# include <boost/thread.hpp>],
            [boost::barrier wait(1);]
        )],
        [boost_thread_found='yes'],
        [boost_thread_found='no']
    )
fi
if test "$BOOST_DIR" != '' && test "$boost_thread_found" == 'no' ; then
    bthread_lib="$BOOST_DIR/lib/libboost_thread.so"
    LDFLAGS="$LDFLAGS_save $bthread_lib"
    AC_LINK_IFELSE(
        [AC_LANG_PROGRAM(
            [# include <boost/thread.hpp>],
            [boost::barrier wait(1);]
        )],
        [boost_thread_found='yes'],
        [boost_thread_found='no']
    )
fi
if test "$BOOST_DIR" != '' && test "$boost_thread_found" == 'no' ; then
    bthread_lib="$BOOST_DIR/lib/libboost_thread.a"
    LDFLAGS="$LDFLAGS_save $bthread_lib"
    AC_LINK_IFELSE(
        [AC_LANG_PROGRAM(
            [# include <boost/thread.hpp>],
            [boost::barrier wait(1);]
        )],
        [boost_thread_found='yes'],
        [boost_thread_found='no']
    )
fi
AM_CONDITIONAL(CppAD_BTHREAD, test "$boost_thread_found" = 'yes')
if test "$boost_thread_found" == 'yes' ; then
    AC_SUBST(BTHREAD_LIB,"$bthread_lib")
    AC_MSG_RESULT([boost::thread library... yes])
else
    AC_SUBST(BTHREAD_LIB,'')
    AC_MSG_RESULT([boost::thread library... no])
fi
LDFLAGS=$LDFLAGS_save
CXXFLAGS=$CXXFLAGS_save
AC_LANG_POP([C++])
#
dnl --------------------------------------------------------------------------
dnl Library directories
if test -e $IPOPT_DIR/lib/coin ; then
    IPOPT_LIBS="-L $IPOPT_DIR/lib/coin -L $IPOPT_DIR/lib/coin-or/ThirdParty"
    IPOPT_LD_PATH="$IPOPT_DIR/lib/coin:$IPOPT_DIR/lib/coin-or/ThirdParty"
else
    IPOPT_LIBS="-L $IPOPT_DIR/lib"
    IPOPT_LD_PATH="$IPOPT_DIR/lib"
fi
AC_SUBST(CPPAD_IPOPT_LD_PATH, "$IPOPT_LD_PATH")
#
AC_SUBST(CPPAD_IPOPT_LIBS, "$IPOPT_LIBS -lipopt")
if test "$have_pkg_config" == "yes"; then
    dnl set CPPAD_IPOPT_LIBS
    PKG_CONFIG_PATH="$IPOPT_DIR/lib/pkgconfig:$IPOPT_DIR/share/pkgconfig"
    PKG_CONFIG_PATH="$PKG_CONFIG_PATH:$IPOPT_DIR/lib64/pkgconfig"
    export PKG_CONFIG_PATH
    if pkg-config --libs ipopt >& /dev/null ; then
        CPPAD_IPOPT_LIBS=`pkg-config --libs ipopt`
    else
        dnl check for blas library
        AC_LANG_PUSH([Fortran])
        AC_CHECK_LIB(
            [blas],
            [ddot],
            [CPPAD_IPOPT_LIBS="$CPPAD_IPOPT_LIBS -lblas"],
        )
        dnl check for lapack library
        AC_CHECK_LIB(
            [lapack],
            [dgesv],
            [CPPAD_IPOPT_LIBS="$CPPAD_IPOPT_LIBS -llapack"],
        )
        AC_LANG_POP([Fortran])
    fi
else
    dnl check for blas library
    AC_LANG_PUSH([Fortran])
    AC_CHECK_LIB(
        [blas],
        [ddot],
        [CPPAD_IPOPT_LIBS="$CPPAD_IPOPT_LIBS -lblas"],
    )
    dnl check for lapack library
    AC_CHECK_LIB(
        [lapack],
        [dgesv],
        [CPPAD_IPOPT_LIBS="$CPPAD_IPOPT_LIBS -llapack"],
    )
    AC_LANG_POP([Fortran])
fi
AC_MSG_RESULT([cppad_ipopt_nlp libraries: $CPPAD_IPOPT_LIBS])
dnl --------------------------------------------------------------------------
dnl Set the values of the names needed by the files
dnl pkgconfig/cppad.pc.in and pkgconfig/cppad-uninstalled.pc.in.
dnl
dnl These settings are an attempt to be the same as in CMakeLists.txt
dnl and pkgconfig/CMakeLists.txt with the following restrictions:
dnl 1. The default prefix for the autoconf install is $HOME.
dnl 2. autoconf configuration does not allow for Colpack.
dnl 3. Assume that libraries are being installed in prefix/lib.
dnl 4. Only install pkgconfig files below prefix/lib (not in datadir).
dnl
dnl cppad_description
AC_SUBST(cppad_description, "Differentiation of C++ Algorithms")
dnl
dnl cppad_version
AC_SUBST(cppad_version, ${PACKAGE_VERSION} )
dnl
dnl cppad_url
AC_SUBST(cppad_url, "http://www.coin-or.org/CppAD")
dnl
dnl cppad_pkgconfig_cflags_uninstalled
AC_SUBST(cppad_pkgconfig_cflags_uninstalled, "-I$abs_top_srcdir")
dnl
dnl cppad_pkgconfig_cflags
if test "$prefix" == "NONE" ; then
    value="-I$HOME/include"
else
    value="-I$prefix/include"
fi
if test "$POSTFIX_DIR" != "" ; then
    value="$value/$POSTFIX_DIR"
fi
AC_SUBST(cppad_pkgconfig_cflags, "$value")
dnl
dnl cppad_pkgconfig_libs_uninstalled
if test "$IPOPT_DIR" == "" ; then
    value=""
else
    value="-L$abs_top_builddir/cppad_ipopt/src -lcppad_ipopt"
fi
AC_SUBST(cppad_pkgconfig_libs_uninstalled, "$value")
dnl
dnl cppad_pkgconfig_libs
if test "$prefix" == "NONE" ; then
    value="-L$HOME/lib"
else
    value="-L$prefix/lib"
fi
if test "$POSTFIX_DIR" != "" ; then
    value="$value/$POSTFIX_DIR"
fi
if test "$IPOPT_DIR" == "" ; then
    value=""
else
    value="$value -lcppad_ipopt"
fi
AC_SUBST(cppad_pkgconfig_libs, "$value")
dnl
dnl cppad_pkgconfig_requires
dnl cppad_pkgconfig_requires_uninstalled
if test "$IPOPT_DIR" == "" ; then
    value=""
else
    value="ipopt"
fi
AC_SUBST(cppad_pkgconfig_requires, "$value")
AC_SUBST(cppad_pkgconfig_requires_uninstalled, "$value")
dnl
dnl -------------------------------------------------------------------------
dnl names set here so */test_one.sh.in works both with cmake and autoconf
dnl (no longer used ?)
AC_SUBST(adolc_prefix,      ${ADOLC_DIR} )
AC_SUBST(eigen_prefix,      ${EIGEN_DIR} )
AC_SUBST(ipopt_prefix,      ${IPOPT_DIR} )
dnl
dnl -----------------------------------------------------------------------
dnl AC_CONFIG_FILES(file-list) for each file in the list, configure will
dnl read file.in, do its substitutions, and create file
AC_CONFIG_FILES([
    include/cppad/configure.hpp
    cppad_ipopt/example/test.sh
    cppad_ipopt/speed/test.sh
    cppad_ipopt/test/test.sh
    example/ipopt_solve/test.sh
    pkgconfig/cppad.pc
    pkgconfig/cppad-uninstalled.pc

    makefile
    cppad_ipopt/src/makefile
    cppad_ipopt/example/makefile
    cppad_ipopt/speed/makefile
    cppad_ipopt/test/makefile
    cppad_lib/makefile
    example/general/makefile
    example/abs_normal/makefile
    example/atomic_two/makefile
    example/atomic_three/makefile
    example/chkpoint_two/makefile
    example/ipopt_solve/makefile
    example/optimize/makefile
    example/sparse/makefile
    example/utility/makefile
    example/get_started/makefile
    include/makefile
    introduction/makefile
    example/multi_thread/makefile
    example/print_for/makefile
    speed/adolc/makefile
    speed/cppad/makefile
    speed/double/makefile
    speed/example/makefile
    speed/fadbad/makefile
    speed/sacado/makefile
    speed/src/makefile
    speed/xpackage/makefile
    test_more/compare_c/makefile
    test_more/general/makefile
    test_more/deprecated/makefile
    test_more/deprecated/atomic_two/makefile
    test_more/deprecated/chkpoint_one/makefile
])
dnl END AC_CONFIG_FILES

dnl copy config.hpp to source directory
AC_CONFIG_COMMANDS(
    [configure.hpp],
    [cp include/cppad/configure.hpp  \
        $abs_top_srcdir/include/cppad/configure.hpp],
[abs_top_srcdir=$abs_top_srcdir]
)
dnl scripts that set library path before running test
AC_CONFIG_COMMANDS(
    [example_ipopt_solve_test.sh],
    [chmod +x example/ipopt_solve/test.sh],
    []
)
AC_CONFIG_COMMANDS(
    [cppad_ipopt_example_test.sh],
    [chmod +x cppad_ipopt/example/test.sh],
    []
)
AC_CONFIG_COMMANDS(
    [cppad_ipopt_speed_test.sh],
    [chmod +x cppad_ipopt/speed/test.sh],
    []
)
AC_CONFIG_COMMANDS(
    [cppad_ipopt_test_test.sh],
    [chmod +x cppad_ipopt/test/test.sh],
    []
)

dnl create all the requested output files
AC_OUTPUT
