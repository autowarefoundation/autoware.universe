# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2019 Joao Leal
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

SET(_outdir "${CMAKE_BINARY_DIR}/test/pcheaders")
FILE(MAKE_DIRECTORY "${_outdir}")

DEFINE_PROPERTY(TARGET
        PROPERTY PRECOMPILED_HEADER_OUTPUT_DIR
        BRIEF_DOCS "The path to the output of a precompiled header"
        FULL_DOCS "The path to the output of a precompiled header" "sss")

DEFINE_PROPERTY(TARGET
        PROPERTY PRECOMPILED_HEADERS
        BRIEF_DOCS "The list of a precompiled headers"
        FULL_DOCS "The list of a precompiled headers" "sss")


FUNCTION(precompile_header target_name)

    IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")

        GET_DIRECTORY_PROPERTY(_directory_flags COMPILE_DEFINITIONS)
        FOREACH (item ${_directory_flags})
            LIST(APPEND _compiler_FLAGS "-D${item}")
        ENDFOREACH ()

        SET(SYSTEM_CFLAGS ${_compiler_FLAGS})

        STRING(TOUPPER "CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}" _flags_var_name)
        SET(_compiler_FLAGS ${${_flags_var_name}})

        GET_DIRECTORY_PROPERTY(_directory_flags INCLUDE_DIRECTORIES)

        FOREACH (item ${_directory_flags})
            LIST(APPEND _compiler_FLAGS "-I${item}")
        ENDFOREACH ()

        SEPARATE_ARGUMENTS(_compiler_FLAGS)
        STRING(REPLACE "(" "\\(" _compiler_FLAGS_STR "${_compiler_FLAGS}")
        STRING(REPLACE ")" "\\)" _compiler_FLAGS_STR "${_compiler_FLAGS_STR}")
        MESSAGE(STATUS "${CMAKE_CXX_COMPILER} -DPCHCOMPILE ${_compiler_FLAGS_STR} -x c++-header -o ${_output} ${_source}")

        SET(_outputs)

        FOREACH (header_file ${ARGN})
            GET_FILENAME_COMPONENT(_name ${header_file} NAME)
            GET_FILENAME_COMPONENT(_dir ${header_file} DIRECTORY)

            # Find the complete path to the header file
            SET(header_file_full_path )
            FOREACH (item ${_directory_flags})
                LIST(APPEND _compiler_FLAGS "-I${item}")
                IF("${header_file_full_path}" STREQUAL "" AND EXISTS "${item}/${header_file}")
                    SET(header_file_full_path "${item}/${header_file}")
                ENDIF()
            ENDFOREACH ()

            IF("${header_file_full_path}" STREQUAL "")
                MESSAGE(ERROR "Failed to find the complete path for ${header_file} using the include directories")
            ELSE()
                MESSAGE(STATUS "Found ${header_file} at ${header_file_full_path}")
            ENDIF()

            SET(_output_dir "${_outdir}/${_name}.build")
            FILE(MAKE_DIRECTORY "${_output_dir}")
            SET(_output "${_output_dir}/${_name}.gch")

            ADD_CUSTOM_COMMAND(OUTPUT ${_output}
                    COMMAND ${CMAKE_CXX_COMPILER} ${_compiler_FLAGS_STR} -std=gnu++11 -x c++-header -o ${_output} ${header_file_full_path}
                    DEPENDS ${_source}
                    COMMENT "Precompiling header ${header_file}")

            LIST(APPEND _outputs "${_output}")
        ENDFOREACH ()

        ADD_CUSTOM_TARGET("${target_name}" DEPENDS ${_outputs})

        SET_PROPERTY(TARGET "${target_name}"
                PROPERTY PRECOMPILED_HEADER_OUTPUT_DIR ${_output_dir})

        SET_PROPERTY(TARGET "${target_name}"
                PROPERTY PRECOMPILED_HEADERS ${target_name})

    ENDIF()

ENDFUNCTION()

################################################################################

FUNCTION(depends_on_precompile_header test_target_name header_target)
    IF ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        GET_TARGET_PROPERTY(_pcheader_dir "${header_target}" PRECOMPILED_HEADER_OUTPUT_DIR)
        GET_TARGET_PROPERTY(_pcheaders "${header_target}" PRECOMPILED_HEADERS)

        SET(_includes)
        FOREACH (item ${_pcheaders})
            LIST(APPEND _includes "-include ${item}")
        ENDFOREACH ()


        ADD_DEPENDENCIES(${test_target_name} "${header_target}")
        SET_TARGET_PROPERTIES(${test_target_name} PROPERTIES COMPILE_FLAGS "-Winvalid-pch ${_includes}") #-H -I${_pcheader}
    ENDIF ()

ENDFUNCTION()