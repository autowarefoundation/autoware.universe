# Copyright (C) 2010 Olivier Stasse, JRL, CNRS, 2010
#
# Original source:
# https://github.com/jrl-umi3218/jrl-cmakemodules/blob/master/cmake_uninstall.cmake.in
#
# Contributor
# https://jcarpent.github.io/
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
set(MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt")

if(EXISTS ${MANIFEST})
  file(STRINGS ${MANIFEST} files)
  foreach(file ${files})
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
  endforeach(file)
else()
  message(FATAL_ERROR "Cannot find install manifest: ${MANIFEST}")
endif()
