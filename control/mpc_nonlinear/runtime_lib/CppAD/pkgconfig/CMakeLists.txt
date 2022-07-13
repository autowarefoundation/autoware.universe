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
# Construct cppad.pc from cppad.pc.in; see
# http://people.freedesktop.org/~dbn/pkg-config-guide.html
# -----------------------------------------------------------------------------
# $begin pkgconfig$$ $newlinech #$$
# $spell
#   CppAD
#   config
#   libdir
#   pkgconfig
#   datadir
#   cppad
#   builddir
#   cmake
#   cflags
#   uninstalled
#   includedir
#   libdirs
# $$
#
# $section CppAD pkg-config Files$$
#
# $head Purpose$$
# The $code pkg-config$$ program helps with the used if installed libraries
# and include files; see its
# $href%https://people.freedesktop.org/~dbn/pkg-config-guide.html%guide%$$
# for more information on writing an using pkg-config files.
#
# $head cppad.pc$$
#
# $subhead PKG_CONFIG_PATH$$
# The $code cppad.pc$$ file is installed in the following two directories:
# $codei%
#   %prefix%/%libdir%/pkgconfig
#   %prefix%/%datadir%/pkgconfig
# %$$
# where $icode prefix$$ is $cref/cppad_prefix/cmake/cppad_prefix/$$,
# $icode libdir$$ is the first entry in
# $cref/cmake_install_libdirs/cmake/cmake_install_libdirs/$$,
# and # $icode datadir$$ is
# $cref/cmake_install_datadir/cmake/cmake_install_datadir/$$.
# In order to use $code pkg-config$$,
# one of these directories must your $code PKG_CONFIG_PATH$$.
#
# $subhead Compile Flags$$
# The necessary flags for compiling code that includes CppAD can
# be obtained with the command
# $codep
#     pkg-config --cflags cppad
# $$
#
# $subhead Link Flags$$
# The flags for linking can be obtains with the command
# $codep
#     pkg-config --libs cppad
# $$
#
# $subhead Extra Definitions$$
# The $code cppad.pc$$ file contains the text:
# $codei%
#   prefix=%prefix%
#   exec_prefix=%exec_prefix%
#   includedir=%includedir%
#   libdir=%libdir%
# %$$
# where $icode prefix$$, $icode exec_prefix$$, $icode includedir$$, and
# $icode libdir$$ are the
# $href%
#   https://www.gnu.org/prep/standards/html_node/Directory-Variables.html%
#   gnu installation variables
# %$$.
#
# $head cppad-uninstalled.pc$$
#
# $subhead PKG_CONFIG_PATH$$
# The $code cppad-uninstalled.pc$$ file is located in the following directory:
# $codei%
#   %builddir%/pkgconfig
# %$$
# where $icode builddir$$ is the directory where the $cref cmake$$ command
# is executed.
#
# $subhead Compile Flags$$
# The necessary flags for compiling code that includes CppAD,
#  before CppAD is installed, can be obtained with the command
# $codep
#     pkg-config --cflags cppad-uninstalled
# $$
#
# $subhead Link Flags$$
# The flags for linking can be obtains with the command
# $codep
#     pkg-config --libs cppad-uninstalled
# $$
#
# $subhead Extra Definitions$$
# The $code cppad-uninstalled.pc$$ file has the same extra variables
# as the $code cppad.pc$$ file.
#
# $end
# ----------------------------------------------------------------------------
#
# The following variables are used by cppad.pc.in
# and set in the top source CMakeLists.txt:
#   cppad_prefix,
#   cppad_description,
#   cppad_version,
#   cppad_url,
#   cppad_lib
# The other variables used by cppad.pc.in are set below
# Note that cppad_SOURCE_DIR is used by cppad-uninstalled.pc
# and is set by cmake to the top soruce directory.
#
# cppad_includedir
LIST(GET cmake_install_includedirs 0 cppad_includedir)
#
# cppad_libdir
LIST(GET cmake_install_libdirs 0 cppad_libdir)
#
# add_to_set
MACRO(add_to_set variable_name element)
    IF( "${${variable_name}}" STREQUAL "" )
        SET(${variable_name} ${element} )
    ELSE( "${${variable_name}}" STREQUAL "" )
        LIST(FIND ${variable_name} ${element} index)
        IF( index EQUAL -1 )
            SET(${variable_name} "${${variable_name}} ${element}")
        ENDIF( index EQUAL -1 )
    ENDIF( "${${variable_name}}" STREQUAL "" )
ENDMACRO(add_to_set variable_name element)
# -----------------------------------------------------------------------------
# initialize
SET(cppad_libdir_list "-L${cppad_prefix}/${cppad_libdir}")
SET(cppad_lib_list    "-l${cppad_lib}")
SET(cppad_requires         "")
SET(cppad_libs_private     "")
SET(cppad_requires_private "")
#
# Colpack does not have a pkgconfig file.
SET(colpack_libdir     NOTFOUND)
IF( cppad_has_colpack )
    FOREACH(dir ${cmake_install_libdirs})
        FILE(GLOB file_list "${colpack_prefix}/${dir}/libColPack.*" )
        IF( file_list )
            SET(colpack_libdir "${colpack_prefix}/${dir}")
        ENDIF( file_list )
    ENDFOREACH(dir ${cmake_install_libdirs})
    IF( NOT colpack_libdir )
        MESSAGE(FATAL_ERROR "Cannit find libColPack.* below ${colpack_prefix}")
    ENDIF( NOT colpack_libdir )
    add_to_set(cppad_libdir_list "-L${colpack_libdir}")
    add_to_set(cppad_lib_list    "-lColPack")
ENDIF( cppad_has_colpack )
#
# Ipopt has a pkgconfig file.
IF( cppad_has_ipopt )
    SET(cppad_requires         "${cppad_requires} ipopt")
    add_to_set(cppad_lib_list  "-lcppad_ipopt" )
ENDIF( cppad_has_ipopt )
# -----------------------------------------------------------------------------
# cppad.pc
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/cppad.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc
    @ONLY
)
# cppad-uninstalled.pc
CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/cppad-uninstalled.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/cppad-uninstalled.pc
    @ONLY
)
# During install copy cppad.pc to datadir
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc
    DESTINATION ${cppad_abs_datadir}/pkgconfig
)
# During install also copy cppad.pc to libdir
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/cppad.pc
    DESTINATION ${cppad_abs_libdir}/pkgconfig
)
