set (DISTRIB_DIR "${CMAKE_BINARY_DIR}/distrib")
set (DISTRIB_NAME "${DISTRIB_DIR}/${PACKAGE_DIR}")
add_custom_target (prep-source
  COMMAND ${CMAKE_MAKE_PROGRAM} package_source
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${DISTRIB_NAME}
  COMMAND ${CMAKE_COMMAND} -E copy_directory
  _CPack_Packages/Linux-Source/TGZ/${PACKAGE_DIR} ${DISTRIB_NAME}
  COMMAND cd ${DISTRIB_NAME} &&
  find * -type f | sort -u > ${DISTRIB_DIR}/files.1 &&
  ( cd ${PROJECT_SOURCE_DIR} && git ls-files ) |
  sort -u > ${DISTRIB_DIR}/files.2 &&
  comm -23 ${DISTRIB_DIR}/files.[12] | xargs -r -d '\\n' rm
  # Remove now empty directories
  COMMAND for p in 1 2 3 4 5\; do
  find ${DISTRIB_NAME} -type d -empty -print0 | xargs -0r rmdir\; done
  COMMAND ${CMAKE_COMMAND} -E rm -f autogen.done)
add_custom_command (OUTPUT autogen.done
  COMMAND cd ${DISTRIB_NAME} && ${PROJECT_SOURCE_DIR}/autogen.sh &&
  touch ${PROJECT_BINARY_DIR}/autogen.done
  DEPENDS prep-source autogen.sh configure.ac
  Makefile.am src/Makefile.am include/Makefile.am tools/Makefile.am
  doc/Makefile.am man/Makefile.am cmake/Makefile.am
  examples/Makefile.am tests/Makefile.am)
add_dependencies (distrib-man prep-source)
add_custom_target (distrib-all DEPENDS distrib-man autogen.done)
add_custom_command (TARGET distrib-all
  COMMAND cd ${DISTRIB_NAME} && echo ${PROJECT_VERSION} > VERSION &&
  chmod -R g-w .)
add_custom_target (dist
  COMMAND
  cd ${DISTRIB_DIR} &&
  find ${PACKAGE_DIR} -type f | tar cfzT ${PACKAGE_NAME}.tar.gz -
  COMMAND
  rm -f ${DISTRIB_DIR}/${PACKAGE_NAME}.zip &&
  cd ${DISTRIB_DIR} &&
  find ${PACKAGE_DIR} -type f | zip -q ${PACKAGE_NAME}.zip -@
  COMMENT "created distrib/${PACKAGE_NAME}.{tar.gz,zip}")
add_dependencies (dist distrib-all)

if (RSYNC)
  set (USER karney)
  set (DATATOP $ENV{HOME}/web/geographiclib-files)
  set (DATAROOT ${DATATOP}/distrib-C++)
  set (DOCTOP $ENV{HOME}/web/geographiclib-web)
  set (DOCROOT ${DOCTOP}/htdocs/C++)
  set (CGIROOT ${DOCTOP}/cgi-bin)
  set (GEOIDROOT ${DOCTOP}/geoids)
  set (FRSDEPLOY ${USER}@frs.sourceforge.net:/home/frs/project/geographiclib)
  set (WEBDEPLOY ${USER},geographiclib@web.sourceforge.net:.)

  add_custom_target (stage-dist
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      ${DISTRIB_DIR}/${PACKAGE_NAME}.tar.gz
      ${DISTRIB_DIR}/${PACKAGE_NAME}.zip
      ${PROJECT_SOURCE_DIR}/data-distrib/distrib-C++/)
  add_dependencies (stage-dist dist)

  if (BUILD_DOCUMENTATION)
    add_custom_target (stage-doc
      COMMAND ${RSYNC} --delete -a doc/html/ ${DOCROOT}/${PROJECT_VERSION}/)
    add_dependencies (stage-doc doc)
  endif ()

  add_custom_target (deploy-dist
    COMMAND
      ${RSYNC} --delete -av --exclude '*~'
      ${PROJECT_SOURCE_DIR}/data-distrib/distrib-C++/ ${DATAROOT}/ &&
      ${RSYNC} --delete -av
      ${PROJECT_SOURCE_DIR}/data-distrib/00README.md
      ${PROJECT_SOURCE_DIR}/data-distrib/distrib ${DATATOP}/
    COMMAND ${RSYNC} --delete -av
      ${DATAROOT} ${DATATOP}/00README.md ${DATATOP}/distrib
      ${USER}@frs.sourceforge.net:/home/frs/project/geographiclib/)
  add_custom_target (deploy-data
    COMMAND
      ${RSYNC} --delete -av --exclude '*~'
      ${PROJECT_SOURCE_DIR}/data-distrib/*-distrib ${DATATOP}/
    COMMAND ${RSYNC} --delete -av ${DATATOP}/*-distrib
      ${USER}@frs.sourceforge.net:/home/frs/project/geographiclib/)
  add_custom_target (deploy-doc
    COMMAND ${RSYNC} --delete -av -e ssh ${DOCROOT} ${WEBDEPLOY}/htdocs/)

  set (CGI_SCRIPTS
    GeoConvert GeodSolve GeoidEval Planimeter RhumbSolve printlogs Geod)
  set (CGI_UTILS utils)

  add_custom_target (stage-cgi
    COMMAND for f in ${CGI_SCRIPTS}\; do
    install -C $$f.cgi ${CGIROOT}/$$f\; done
    COMMAND for f in ${CGI_UTILS}\; do
    install -C -m 644 $$f.sh ${CGIROOT}/$$f.sh\; done
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/cgi-bin)

  add_custom_target (deploy-cgi
    COMMAND ${RSYNC} --delete -av -e ssh ${CGIROOT} ${GEOIDROOT} ${WEBDEPLOY}/)

endif ()

if (NOT WIN32)
  set (BINARY_EXT "m4|gif|pdf|png|kmz")
  add_custom_target (checktrailingspace
    COMMAND git ls-files |
    grep -E -v '\\.\(${BINARY_EXT}\)$$' |
    xargs grep '[ \t]$$' || true
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Looking for trailing spaces")
  add_custom_target (checktabs
    COMMAND git ls-files |
    grep -E -v '\([Mm]akefile|test-distribution.sh|\\.\(${BINARY_EXT}\)$$\)' |
    xargs grep -l '\t' || true
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Looking for tabs")
  add_custom_target (checkblanklines
    COMMAND git ls-files |
    grep -E -v '\\.\(${BINARY_EXT}\)$$' |
    while read f\; do tr 'X\\n' 'YX' < $$f |
    grep -E '\(^X|XXX|XX$$|[^X]$$\)' > /dev/null && echo $$f\; done || true
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Looking for extra blank lines")

  add_custom_target (sanitize)
  add_dependencies (sanitize checktrailingspace checktabs checkblanklines)
endif ()
