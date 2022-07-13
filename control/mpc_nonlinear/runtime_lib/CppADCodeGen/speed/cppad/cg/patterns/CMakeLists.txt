# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2013 Ciengis
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

INCLUDE_DIRECTORIES("${CMAKE_CURRENT_SOURCE_DIR}" ${LLVM_INCLUDE_DIRS} ${DL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/test")
LINK_DIRECTORIES(${LLVM_LIBRARY_DIRS})
ADD_DEFINITIONS(${LLVM_CFLAGS_NO_NDEBUG} -DLLVM_WITH_NDEBUG=${LLVM_WITH_NDEBUG})

MACRO(add_speed_test name)
  ADD_EXECUTABLE(${name}
                 # sources:
                 "job_speed_listener.cpp"
                 "${name}.cpp")

  IF( UNIX )
      TARGET_LINK_LIBRARIES(${name} ${DL_LIBRARIES})
  ENDIF()

  TARGET_LINK_LIBRARIES(${name}
                        ${CLANG_LIBS}
                        ${LLVM_MODULE_LIBS}
                        ${LLVM_LDFLAGS})

ENDMACRO()

add_speed_test("speed_plugflow")

add_speed_test("speed_collocation")


################################################################################
# Execute benchmark for plugflow
################################################################################
FOREACH(nCstr 100 90 80 70 60 50 40 30 20 10)
   SET(outputStatFile "speed_plugflow_stat_${nCstr}.txt")
   SET(outputDataFile "speed_plugflow_data_${nCstr}.txt")
   LIST(APPEND outputFiles ${outputStatFile} ${outputDataFile})
   ADD_CUSTOM_COMMAND(OUTPUT ${outputStatFile} ${outputDataFile}
                      COMMAND speed_plugflow ${nCstr} > ${outputStatFile} 2> ${outputDataFile}
                      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
ENDFOREACH()

ADD_CUSTOM_TARGET(benchmark_plugflow 
                  DEPENDS ${outputFiles})

################################################################################
# Execute benchmark for collocation
################################################################################
SET(outputFiles "")

FOREACH(nCstr 50 30 10)
   FOREACH(nTimeInt 50 40 30 20 10 5)
      SET(outputStatFile "speed_collocation_stat_${nTimeInt}int_${nCstr}el.txt")
      SET(outputDataFile "speed_collocation_data_${nTimeInt}int_${nCstr}el.txt")
      LIST(APPEND outputFiles ${outputStatFile} ${outputDataFile})
      ADD_CUSTOM_COMMAND(OUTPUT ${outputStatFile} ${outputDataFile}
                         COMMAND speed_collocation ${nTimeInt} ${nCstr} > ${outputStatFile} 2> ${outputDataFile}
                         WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
   ENDFOREACH()
ENDFOREACH()

ADD_CUSTOM_TARGET(benchmark_collocation
                  DEPENDS ${outputFiles})