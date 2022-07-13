###########################################################################
#                     API documentation with Doxygen
###########################################################################
IF(DOXYGEN_FOUND)
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/doxyfile @ONLY)

    IF(${CREATE_DOXYGEN_DOC})
        SET(DOC_TARGET_ALL "ALL")
    ELSE()
        SET(DOC_TARGET_ALL "")
    ENDIF()

    ADD_CUSTOM_TARGET(doc ${DOC_TARGET_ALL} 
                      ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/doxyfile
                      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
                      COMMENT "Generating API documentation with Doxygen" VERBATIM)

    IF(${CREATE_DOXYGEN_DOC})
      INSTALL(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/html"
              DESTINATION "${install_doc_location}")
    ENDIF()

ENDIF()
