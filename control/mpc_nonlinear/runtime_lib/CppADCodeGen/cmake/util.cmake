# --------------------------------------------------------------------------
#  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
#    Copyright (C) 2016 Ciengis
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
# Create links/copies
################################################################################
IF(NOT DEFINED COPY_OR_LINK)
  IF(UNIX) 
    SET(COPY_OR_LINK create_symlink)
  ELSE()
    SET(COPY_OR_LINK copy)
  ENDIF()
ENDIF()

MACRO(link_file from_path to_path)
    GET_FILENAME_COMPONENT(from_path_abs "${from_path}" ABSOLUTE)
    GET_FILENAME_COMPONENT(to_path_abs "${to_path}" ABSOLUTE)

    IF(NOT "${from_path_abs}" STREQUAL "${to_path_abs}")
        ADD_CUSTOM_COMMAND(OUTPUT "${to_path}"
                           COMMAND ${CMAKE_COMMAND} ARGS -E ${COPY_OR_LINK} "${from_path}" "${to_path}"
                           DEPENDS "${from_path}"
                           COMMENT Creating symbolic link)
    ENDIF()
ENDMACRO()
