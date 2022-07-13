# - try to find cppcheck tool
#
# Cache Variables:
#  CPPCHECK_EXECUTABLE
#
# Non-cache variables you might use in your CMakeLists.txt:
#  CPPCHECK_FOUND
#  CPPCHECK_POSSIBLEERROR_ARG
#  CPPCHECK_UNUSEDFUNC_ARG
#  CPPCHECK_STYLE_ARG
#  CPPCHECK_QUIET_ARG
#  CPPCHECK_INCLUDEPATH_ARG
#  CPPCHECK_FAIL_REGULAR_EXPRESSION
#  CPPCHECK_WARN_REGULAR_EXPRESSION
#  CPPCHECK_MARK_AS_ADVANCED - whether to mark our vars as advanced even
#    if we don't find this program.
#
# Requires these CMake modules:
#  FindPackageHandleStandardArgs (known included with CMake >=2.6.2)
#
# Joao Leal:
# 2013 Joao Leal <joao.leal@ciengis.com> 
# removed references to old cppcheck style arguments
#
# Original Author:
# 2009-2010 Ryan Pavlik <rpavlik@iastate.edu> <abiryan@ryand.net>
# http://academic.cleardefinition.com
# Iowa State University HCI Graduate Program/VRAC
#
# Copyright Iowa State University 2009-2010.
# Distributed under the Boost Software License, Version 1.0.
# (See accompanying file LICENSE_1_0.txt or copy at
# http://www.boost.org/LICENSE_1_0.txt)

FILE(TO_CMAKE_PATH "${CPPCHECK_ROOT_DIR}" CPPCHECK_ROOT_DIR)
SET(CPPCHECK_ROOT_DIR "${CPPCHECK_ROOT_DIR}" CACHE PATH "Path to search for cppcheck")

# cppcheck app bundles on Mac OS X are GUI, we want command line only
SET(_oldappbundlesetting ${CMAKE_FIND_APPBUNDLE})
SET(CMAKE_FIND_APPBUNDLE NEVER)

IF(CPPCHECK_EXECUTABLE AND NOT EXISTS "${CPPCHECK_EXECUTABLE}")
	SET(CPPCHECK_EXECUTABLE "notfound" CACHE PATH FORCE "")
ENDIF()

# If we have a custom path, look there first.
IF(CPPCHECK_ROOT_DIR)
	FIND_PROGRAM(CPPCHECK_EXECUTABLE
		NAMES
		cppcheck
		cli
		PATHS
		"${CPPCHECK_ROOT_DIR}"
		PATH_SUFFIXES
		cli
		NO_DEFAULT_PATH)
ENDIF()

FIND_PROGRAM(CPPCHECK_EXECUTABLE NAMES cppcheck)

# Restore original setting for appbundle finding
SET(CMAKE_FIND_APPBUNDLE ${_oldappbundlesetting})

IF(CPPCHECK_EXECUTABLE)
	SET(CPPCHECK_STYLE_ARG "--enable=style")
	SET(CPPCHECK_UNUSEDFUNC_ARG "--enable=unusedFunctions")
	SET(CPPCHECK_INFORMATION_ARG "--enable=information")
	SET(CPPCHECK_MISSINGINCLUDE_ARG "--enable=missingInclude")
	SET(CPPCHECK_POSIX_ARG "--enable=posix")
	SET(CPPCHECK_POSSIBLEERROR_ARG "--enable=possibleError")
	SET(CPPCHECK_POSSIBLEERROR_ARG "--enable=all")
	IF(MSVC)
		SET(CPPCHECK_TEMPLATE_ARG --template vs)
		SET(CPPCHECK_FAIL_REGULAR_EXPRESSION "[(]error[)]")
		SET(CPPCHECK_WARN_REGULAR_EXPRESSION "[(]style[)]")
	ELSEIF(CMAKE_COMPILER_IS_GNUCXX)
		SET(CPPCHECK_TEMPLATE_ARG --template gcc)
		SET(CPPCHECK_FAIL_REGULAR_EXPRESSION " error: ")
		SET(CPPCHECK_WARN_REGULAR_EXPRESSION " style: ")
	ELSE()
		SET(CPPCHECK_TEMPLATE_ARG --template gcc)
		SET(CPPCHECK_FAIL_REGULAR_EXPRESSION " error: ")
		SET(CPPCHECK_WARN_REGULAR_EXPRESSION " style: ")
	ENDIF()

	SET(CPPCHECK_QUIET_ARG "--quiet")
	SET(CPPCHECK_INCLUDEPATH_ARG "-I")

ENDIF()

SET(CPPCHECK_ALL
	"${CPPCHECK_EXECUTABLE} ${CPPCHECK_POSSIBLEERROR_ARG} ${CPPCHECK_UNUSEDFUNC_ARG} ${CPPCHECK_STYLE_ARG} ${CPPCHECK_QUIET_ARG} ${CPPCHECK_INCLUDEPATH_ARG} some/include/path")

INCLUDE(FindPackageHandleStandardArgs)
find_package_handle_standard_args(cppcheck
                                  DEFAULT_MSG
                                  CPPCHECK_ALL
                                  CPPCHECK_EXECUTABLE
                                  CPPCHECK_POSSIBLEERROR_ARG
                                  CPPCHECK_UNUSEDFUNC_ARG
                                  CPPCHECK_STYLE_ARG
                                  CPPCHECK_INCLUDEPATH_ARG
                                  CPPCHECK_QUIET_ARG)

IF(CPPCHECK_FOUND OR CPPCHECK_MARK_AS_ADVANCED)
	MARK_AS_ADVANCED(CPPCHECK_ROOT_DIR)
ENDIF()

MARK_AS_ADVANCED(CPPCHECK_EXECUTABLE)
