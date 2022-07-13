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
ADD_SUBDIRECTORY(dynamiclib)

ADD_SUBDIRECTORY(lang/c)

IF(PDFLATEX_COMPILER)
    ADD_SUBDIRECTORY(lang/latex)
ENDIF()

IF(GRAPHVIZ_DOT_PATH)
    ADD_SUBDIRECTORY(lang/dot)
ENDIF()

ADD_SUBDIRECTORY(lang/mathml)

IF ( ENABLE_THREAD_POOL_TESTS )
    ADD_SUBDIRECTORY(threadpool)
ENDIF()

IF(CPPADCG_USE_LLVM)
    ADD_SUBDIRECTORY(llvm)
ENDIF()
