# Copyright (C) 2010 Olivier Stasse, JRL, CNRS, 2010
#
# Original source:
# https://github.com/jrl-umi3218/jrl-cmakemodules/blob/master/cmake_uninstall.cmake.in
#
# Contributor
# https://jcarpent.github.io/
#
#  CppADCodeGen is distributed under multiple licenses:
#
#   - Eclipse Public License Version 1.0 (EPL1), and
#   - GNU General Public License Version 3 (GPL3).
#
# EPL1 terms and conditions can be found in the file "epl-v10.txt", while
# terms and conditions for the GPL3 can be found in the file "gpl3.txt".
#
set(MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt")

if(EXISTS ${MANIFEST})
  file(STRINGS ${MANIFEST} files)
  foreach(_file ${files})
    SET(file "$ENV{DESTDIR}${_file}")
    if(IS_SYMLINK ${file} OR EXISTS ${file})
      message(STATUS "Removing file: '${file}'")

      execute_process(
        COMMAND ${CMAKE_COMMAND} -E remove ${file}
        OUTPUT_VARIABLE rm_out
        RESULT_VARIABLE rm_retval
        )

      if(NOT "${rm_retval}" STREQUAL 0)
        message(FATAL_ERROR "Failed to remove file: '${file}'.")
      endif()
    else()
      message(STATUS "File '${file}' does not exist.")
    endif()
  endforeach(_file)
else()
  message(FATAL_ERROR "Cannot find install manifest: ${MANIFEST}")
endif()
