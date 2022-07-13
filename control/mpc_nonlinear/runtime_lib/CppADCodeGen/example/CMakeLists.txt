# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
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

################################################################################
#                             source_generation_c
################################################################################
ADD_EXECUTABLE(source_generation_c source_generation_c.cpp)
SET(EXAMPLES_COMMAND "COMMAND source_generation_c")
ADD_CUSTOM_TARGET(example_source_generation_c
                  COMMAND source_generation_c)

################################################################################
#                             source_generation_mathml
################################################################################
ADD_EXECUTABLE(source_generation_mathml source_generation_mathml.cpp)
SET(EXAMPLES_COMMAND "COMMAND source_generation_mathml")
ADD_CUSTOM_TARGET(example_source_generation_mathml
                  COMMAND source_generation_mathml)

################################################################################
#                             source_generation_latex
################################################################################
ADD_EXECUTABLE(source_generation_latex source_generation_latex.cpp)
SET(EXAMPLES_COMMAND "COMMAND source_generation_latex")
ADD_CUSTOM_TARGET(example_source_generation_latex
                  COMMAND source_generation_latex)

IF(PDFLATEX_COMPILER)
    ADD_DEFINITIONS(-DPDFLATEX_COMPILER="${PDFLATEX_COMPILER}")
ENDIF()

link_file("${CMAKE_CURRENT_SOURCE_DIR}/resources"
          "${CMAKE_CURRENT_BINARY_DIR}/resources")

# copy template (if required)
ADD_CUSTOM_TARGET(link_or_copy_resources
                  DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/resources")
ADD_DEPENDENCIES(source_generation_latex link_or_copy_resources)

################################################################################
#                             source_generation_dot
################################################################################
ADD_EXECUTABLE(source_generation_dot source_generation_dot.cpp)
SET(EXAMPLES_COMMAND "COMMAND source_generation_dot")
ADD_CUSTOM_TARGET(example_source_generation_dot
                  COMMAND source_generation_dot)

IF(GRAPHVIZ_DOT_PATH)
    ADD_DEFINITIONS(-DGRAPHVIZ_DOT_PATH="${GRAPHVIZ_DOT_PATH}")
ENDIF()

IF(UNIX)
    ############################################################################
    #                                  dynamic_linux
    ############################################################################
    ADD_EXECUTABLE(dynamic_linux dynamic_linux.cpp)
    SET(EXAMPLES_COMMAND "${EXAMPLES_COMMAND} COMMAND dynamic_linux")
    TARGET_LINK_LIBRARIES(dynamic_linux ${DL_LIBRARIES})
    ADD_CUSTOM_TARGET(example_dynamic_linux
                      COMMAND dynamic_linux)

    ############################################################################
    #                                  atomic
    ############################################################################
    ADD_EXECUTABLE(atomic atomic.cpp)
    SET(EXAMPLES_COMMAND "${EXAMPLES_COMMAND} COMMAND atomic")
    TARGET_LINK_LIBRARIES(atomic ${DL_LIBRARIES})
    ADD_CUSTOM_TARGET(example_atomic
                      COMMAND atomic)

    ############################################################################
    #                                  LLVM
    ############################################################################
    IF(CPPADCG_USE_LLVM)

        INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${LLVM_INCLUDE_DIRS} ${CLANG_INCLUDE_DIRS} ${DL_INCLUDE_DIRS})
        LINK_DIRECTORIES(${LLVM_LIBRARY_DIRS})
        ADD_DEFINITIONS(${LLVM_CFLAGS_NO_NDEBUG} -DLLVM_WITH_NDEBUG=${LLVM_WITH_NDEBUG})

        ADD_EXECUTABLE(jit_linux jit_linux.cpp)
        SET(EXAMPLES_COMMAND "${EXAMPLES_COMMAND} COMMAND dynamic_linux")

        TARGET_LINK_LIBRARIES(jit_linux ${DL_LIBRARIES})
        ADD_CUSTOM_TARGET(example_jit_linux COMMAND jit_linux)

        TARGET_LINK_LIBRARIES(jit_linux ${LLVM_MODULE_LIBS} ${LLVM_LDFLAGS})

        IF("${LLVM_VERSION_MAJOR}.${LLVM_VERSION_MINOR}" MATCHES "^(${CPPADCG_LLVM_LINK_LIB})$")
            TARGET_LINK_LIBRARIES(jit_linux ${CLANG_LIBS})
        ENDIF()

    ENDIF()

    ############################################################################
    #                                  lagrangian
    ############################################################################
    ADD_EXECUTABLE(lagrangian lagrangian.cpp)
    SET(EXAMPLES_COMMAND "${EXAMPLES_COMMAND} COMMAND lagrangian")
    TARGET_LINK_LIBRARIES(lagrangian ${DL_LIBRARIES})
    ADD_CUSTOM_TARGET(example_lagrangian
                      COMMAND lagrangian)

    ############################################################################
    #                                   patterns
    ############################################################################
    ADD_EXECUTABLE(patterns patterns.cpp)
    SET(EXAMPLES_COMMAND "${EXAMPLES_COMMAND} COMMAND patterns")
    TARGET_LINK_LIBRARIES(patterns ${DL_LIBRARIES})
    ADD_CUSTOM_TARGET(example_patterns
                      COMMAND patterns)
ENDIF(UNIX)


ADD_CUSTOM_TARGET(examples)

ADD_DEPENDENCIES(examples example_source_generation_c)
ADD_DEPENDENCIES(examples example_source_generation_latex)
ADD_DEPENDENCIES(examples example_source_generation_mathml)
ADD_DEPENDENCIES(examples example_source_generation_dot)
IF(UNIX)
    ADD_DEPENDENCIES(examples example_atomic)
    ADD_DEPENDENCIES(examples example_dynamic_linux)
    ADD_DEPENDENCIES(examples example_lagrangian)
ENDIF()
