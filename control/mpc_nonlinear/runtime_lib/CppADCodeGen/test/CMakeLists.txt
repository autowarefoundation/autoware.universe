# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2018 Joao Leal
#    Copyright (C) 2012 Ciengis
#
#  CppADCodeGen is distributed under multiple licenses:
#
#   - Eclipse Public License Version 1.0 (EPL1), and
#   - GNU General Public License Version 3 (GPL3).
#
#  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
#  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
# ----------------------------------------------------------------------------
#
# Author: Joao Leal
#
# ----------------------------------------------------------------------------
SET(CMAKE_BUILD_TYPE DEBUG)

################################################################################
# download and compile GTest
################################################################################
INCLUDE(ExternalProject)

# Set default ExternalProject root directory
SET_DIRECTORY_PROPERTIES(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/test/googletest)

# Add gtest
OPTION(GOOGLETEST_GIT "Download GoogleTest from the GIT repositoty" OFF)
IF(GOOGLETEST_GIT)
  # Download from SVN repository
    ExternalProject_Add(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.8.1
        TIMEOUT 10
        # Force separate output paths for debug and release builds to allow easy
        # identification of correct lib in subsequent TARGET_LINK_LIBRARIES commands
        CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
                   -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
                   -Dgtest_force_shared_crt=ON
        # Disable install step
        INSTALL_COMMAND ""
        # Wrap download, configure and build steps in a script to log output
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        LOG_BUILD ON)
ELSE()
    # find installed in system
    FIND_PATH(GOOGLETEST_SOURCE_DIR NAMES "src/gtest_main.cc"
              HINTS  $ENV{GOOGLETEST_HOME}
                     "/usr/src/gtest/" )
    IF(NOT IS_DIRECTORY ${GOOGLETEST_SOURCE_DIR})
        MESSAGE(FATAL_ERROR "GoogleTest source folder not found")
    ENDIF()

    ExternalProject_Add(
        googletest
        URL ${GOOGLETEST_SOURCE_DIR}
        # Force separate output paths for debug and release builds to allow easy
        # identification of correct lib in subsequent TARGET_LINK_LIBRARIES commands
        CMAKE_ARGS -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_DEBUG:PATH=DebugLibs
                   -DCMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE:PATH=ReleaseLibs
                   -Dgtest_force_shared_crt=ON
        # Disable install step
        INSTALL_COMMAND ""
        # Wrap download, configure and build steps in a script to log output
        LOG_DOWNLOAD ON
        LOG_CONFIGURE ON
        LOG_BUILD ON)
ENDIF()

# Specify include dir
ExternalProject_Get_Property(googletest source_dir)
SET(gtest_source_dir ${source_dir})

# Specify MainTest's link libraries
ExternalProject_Get_Property(googletest binary_dir)
IF(GOOGLETEST_GIT)
    SET(GTEST_LIBS_DIR ${binary_dir}/googlemock/gtest)
ELSE()
    SET(GTEST_LIBS_DIR ${binary_dir})
ENDIF()

INCLUDE_DIRECTORIES("${gtest_source_dir}/googletest/include")

SET(GTEST_LIBRARY_PATH ${GTEST_LIBS_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a)
SET(GTEST_MAIN_PATH    ${GTEST_LIBS_DIR}/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main.a)

ADD_LIBRARY(gtest UNKNOWN IMPORTED)
SET_TARGET_PROPERTIES(gtest PROPERTIES
                      "IMPORTED_LOCATION" "${GTEST_LIBRARY_PATH}"
                      "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
                      "INTERFACE_INCLUDE_DIRECTORIES" "${GTEST_INCLUDE_DIRS}")
ADD_DEPENDENCIES(gtest googletest)

ADD_LIBRARY(gtestmain UNKNOWN IMPORTED)
SET_TARGET_PROPERTIES(gtestmain PROPERTIES
                      "IMPORTED_LOCATION" "${GTEST_MAIN_PATH}"
                      "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
                      "INTERFACE_INCLUDE_DIRECTORIES" "${GTEST_INCLUDE_DIRS}")
ADD_DEPENDENCIES(gtestmain googletest)

################################################################################
# add gtest dependency
################################################################################
FUNCTION(add_gest_link_libraries test_target_name use_main)
    IF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        TARGET_LINK_LIBRARIES(${test_target_name} gtest)
        IF(${use_main})
            TARGET_LINK_LIBRARIES(${test_target_name} gtestmain)
        ENDIF()
        TARGET_LINK_LIBRARIES(${test_target_name} pthread)
    ELSE()
        TARGET_LINK_LIBRARIES(${test_target_name}
                              debug ${GTEST_LIBS_DIR}/DebugLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_FIND_LIBRARY_SUFFIXES}
                              optimized ${GTEST_LIBS_DIR}/ReleaseLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest${CMAKE_FIND_LIBRARY_SUFFIXES})
        IF(${use_main})
            TARGET_LINK_LIBRARIES(${test_target_name}
                                  debug ${GTEST_LIBS_DIR}/DebugLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_FIND_LIBRARY_SUFFIXES}
                                  optimized ${GTEST_LIBS_DIR}/ReleaseLibs/${CMAKE_FIND_LIBRARY_PREFIXES}gtest_main${CMAKE_FIND_LIBRARY_SUFFIXES})
        ENDIF()
  ENDIF()

  # Create dependency of test on googletest
  ADD_DEPENDENCIES(${test_target_name} googletest)
ENDFUNCTION()

################################################################################
# create a temporary folder for a set of tests
################################################################################
FUNCTION(create_temp_folder_target)
    IF(NOT TARGET "create_tmp_folder_${folder_name}")
        GET_FILENAME_COMPONENT(folder_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)

        ADD_CUSTOM_COMMAND(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/tmp"
                           COMMAND ${CMAKE_COMMAND} -E remove_directory "tmp" # make sure tmp is empty
                           COMMAND ${CMAKE_COMMAND} -E make_directory   "tmp"
                           COMMENT "Creating tmp folder")
        ADD_CUSTOM_TARGET("create_tmp_folder_${folder_name}" ALL
                          DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/tmp")
    ENDIF()
ENDFUNCTION()

################################################################################
# add test function
################################################################################
ADD_CUSTOM_TARGET(build_tests)
FUNCTION(add_cppadcg_test source_file)

    GET_FILENAME_COMPONENT(test_target_name "${source_file}" NAME_WE)

    ADD_EXECUTABLE("${test_target_name}"
                   # sources:
                   "${source_file}"
                   ${ARGN})

    GET_FILENAME_COMPONENT(folder_name ${source_file} ABSOLUTE)
    GET_FILENAME_COMPONENT(folder_name ${folder_name} PATH)
    GET_FILENAME_COMPONENT(folder_name ${folder_name} NAME)

    create_temp_folder_target()
    ADD_DEPENDENCIES("${test_target_name}" "create_tmp_folder_${folder_name}")

    # Specify MainTest's link libraries
    add_gest_link_libraries(${test_target_name} ON)

    IF( UNIX )
        TARGET_LINK_LIBRARIES(${test_target_name} ${DL_LIBRARIES})
    ENDIF()

    ADD_TEST(NAME ${test_target_name}
            COMMAND ${test_target_name})

    IF($ENABLE_TEST_CPPCHECKS)
        add_test_cppcheck(${test_target_name} STYLE MISSING_INCLUDE)
    ENDIF()

    IF(VALGRIND_FOUND)
        ADD_TEST(NAME ${test_target_name}_memcheck
                 COMMAND ${GTEST_MEMORYCHECK_COMMAND}
                         --read-var-info=yes
                         --leak-check=full
                         --track-origins=yes
                         --leak-resolution=high
                         --vgdb=no
                         --quiet
                         --error-exitcode=1
                         --log-file=${CMAKE_CURRENT_BINARY_DIR}/valgrind.log.${test_target_name}
                         --num-callers=20
                         ./${test_target_name}
                 COMMENT "Running valgrind on test \"${test_target_name}\"")
    ENDIF()

    ################################################################################
    # precompile headers
    ################################################################################
    IF(USE_PRECOMPILED_HEADERS)
        depends_on_precompile_header(${test_target_name} "CppADCGTest.hpp")
    ENDIF()

    ADD_DEPENDENCIES(build_tests ${test_target_name})

ENDFUNCTION()
################################################################################
# use valgrind if possible
################################################################################
OPTION(USE_VALGRIND "Use valgrind in tests" ON)

IF(${USE_VALGRIND})
    FIND_PACKAGE(Valgrind)

    IF(${VALGRIND_FOUND})
        SET(GTEST_MEMORYCHECK_COMMAND "${VALGRIND_EXECUTABLE}")
        MESSAGE(STATUS "Valgrind found")
    ELSE()
        MESSAGE(WARNING "Valgrind not found")
    ENDIF()
ELSE()
    IF(CMAKE_COMPILER_IS_GNUCXX)
        OPTION(USE_SANITIZE "Use gcc compiler sanitize flags in tests" OFF)

        IF(${USE_SANITIZE})
            SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fsanitize=address -fno-omit-frame-pointer")
            ADD_DEFINITIONS("-fsanitize=address")
            ADD_DEFINITIONS("-DCPPADCG_TEST_SANITIZE")
        ENDIF()
    ENDIF()
ENDIF()

################################################################################
# add tests
################################################################################

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

################################################################################

INCLUDE(${CMAKE_SOURCE_DIR}/cmake/PrecompiledHeaders.cmake)

OPTION(USE_PRECOMPILED_HEADERS "Use precompiled headers in tests" OFF)

################################################################################

IF(USE_PRECOMPILED_HEADERS)
    precompile_header(CppADCGTest.hpp
                      cppad/cg/cppadcg.hpp)
ENDIF()

ADD_SUBDIRECTORY(cppad/cg)
#ADD_SUBDIRECTORY(zoot)

FILE(COPY "${CMAKE_CURRENT_SOURCE_DIR}/valgrind.sh"
     DESTINATION "${CMAKE_CURRENT_BINARY_DIR}")
