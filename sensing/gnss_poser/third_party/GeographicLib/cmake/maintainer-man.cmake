# In non-RELEASE mode the mans pages are in pod format

add_custom_target (man ALL)
set (MANPAGES)
set (USAGE)
set (HTMLMAN)

foreach (TOOL ${TOOLS})
  set (MANPAGES ${MANPAGES} ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.1)
  set (USAGE ${USAGE} ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.usage)
  set (HTMLMAN ${HTMLMAN} ${CMAKE_CURRENT_BINARY_DIR}/${TOOL}.1.html)

  if (MAINTAINER)

    # A maintainer can transform these to man, html, and usage formats.
    add_custom_command (OUTPUT ${TOOL}.1
      COMMAND ${POD2MAN} --center=\"GeographicLib Utilities\"
      --date `date +%Y-%m-%d`
      --release=\"GeographicLib ${PROJECT_VERSION}\"
      ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.pod > ${TOOL}.1
      COMMENT "Building man page for ${TOOL}"
      MAIN_DEPENDENCY ${TOOL}.pod)
    add_custom_command (OUTPUT ${TOOL}.1.html COMMAND
      ${POD2HTML} --title "'${TOOL}(1)'"
      --noindex ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.pod |
      sed
      -e 's%<head>%<head><link href="http://search.cpan.org/s/style.css" rel="stylesheet" type="text/css">%'
      -e 's%<code>\\\([^<>]*\\\)\(\\\(.\\\)\)</code>%<a href="\\1.\\2.html">&</a>%'g > ${TOOL}.1.html
      COMMENT "Building html version of man page for ${TOOL}"
      MAIN_DEPENDENCY ${TOOL}.pod)
    add_custom_command (OUTPUT ${TOOL}.usage
      COMMAND env POD2MAN=${POD2MAN} COL=${COL}
      sh ${CMAKE_CURRENT_SOURCE_DIR}/makeusage.sh
      ${CMAKE_CURRENT_SOURCE_DIR}/${TOOL}.pod ${PROJECT_VERSION}
      > ${TOOL}.usage
      COMMENT "Building usage code for ${TOOL}"
      MAIN_DEPENDENCY ${TOOL}.pod)

  else ()

    # Otherwise, dummy versions of these pages are created from
    # templates dummy.XXX.in.  These dummy versions point to the online
    # documentation.
    configure_file (dummy.usage.in ${TOOL}.usage @ONLY)
    configure_file (dummy.1.in ${TOOL}.1 @ONLY)
    configure_file (dummy.1.html.in ${TOOL}.1.html @ONLY)

  endif ()
endforeach ()

if (MANDIR AND BINDIR)
  install (FILES ${MANPAGES} DESTINATION ${MANDIR}/man1)
endif ()

if (MAINTAINER)
  add_custom_target (manpages ALL DEPENDS ${MANPAGES}
    COMMENT "Building all the man pages")
  add_custom_target (usage ALL DEPENDS ${USAGE}
    COMMENT "Converting the man pages to online usage")
  add_custom_target (htmlman ALL DEPENDS ${HTMLMAN}
    COMMENT "Building html versions of the man pages")
else ()
  add_custom_target (manpages ALL DEPENDS ${MANPAGES}
    COMMENT "Building dummy man pages")
  add_custom_target (usage ALL DEPENDS ${USAGE}
    COMMENT "Converting dummy man pages to online usage")
  add_custom_target (htmlman ALL DEPENDS ${HTMLMAN}
    COMMENT "Building dummy html versions of the man pages")
endif ()

add_dependencies (man manpages usage htmlman)

if (MAINTAINER)
  # The distrib-man target copies the derived documentation files into
  # the source tree.
  add_custom_target (distrib-man)
  add_dependencies (distrib-man man)
  add_custom_command (TARGET distrib-man
    COMMAND
    for f in ${MANPAGES} ${USAGE} ${HTMLMAN}\; do
    install -C -m 644 "$$f" ../distrib/${PACKAGE_DIR}/man\; done
    COMMENT "Installing man documentation page in source tree")
endif ()
