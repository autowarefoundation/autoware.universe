# Build the test programs...

# Use fftw for AreaEst?  Because CMake support for installed fftw
# package is broken, using FFTW required building local copies of the
# library (a separate cmake configure and build for each precision).  So
# default to not using fftw and fall back on kissfftw.  For these
# reasons + the overall complexity of adding a dependency to
# GeographicLib, the DST class uses kissfft (despite it being somewhat
# slower that fftw).
set (USE_FFTW OFF)
if (USE_FFTW)
  if (GEOGRAPHICLIB_PRECISION EQUAL 1)
    set (FFTW_PACKAGE FFTW3f)
  elseif (GEOGRAPHICLIB_PRECISION EQUAL 2)
    set (FFTW_PACKAGE FFTW3)
  elseif (GEOGRAPHICLIB_PRECISION EQUAL 3)
    set (FFTW_PACKAGE FFTW3l)
  elseif (GEOGRAPHICLIB_PRECISION EQUAL 4)
    set (FFTW_PACKAGE FFTW3q)
  else () # GEOGRAPHICLIB_PRECISION EQUAL 5
    set (FFTW_PACKAGE OFF)
  endif ()
  if (FFTW_PACKAGE)
    find_package (${FFTW_PACKAGE})
    if (${FFTW_PACKAGE}_FOUND)
      set (FFTW_FOUND ON)
      set (FFTW_LIBRARIES ${${FFTW_PACKAGE}_LIBRARIES})
      set (FFTW_LIBRARY_DIRS ${${FFTW_PACKAGE}_LIBRARY_DIRS})
      set (FFTW_INCLUDE_DIRS ${${FFTW_PACKAGE}_INCLUDE_DIRS})
    endif ()
  endif ()
endif ()

set (DEVELPROGRAMS
  ProjTest TMTest GeodTest ConicTest NaNTester HarmTest EllipticTest intersect
  ClosestApproach M12zero GeodShort NormalTest)

if (Boost_FOUND AND NOT GEOGRAPHICLIB_PRECISION EQUAL 4)
  # Skip LevelEllipsoid for quad precision because of compiler errors
  # with boost 1.69 and g++ 9.2.1 (Fedora 30).  Problem reported as
  # https://github.com/boostorg/odeint/issues/40
  set (DEVELPROGRAMS ${DEVELPROGRAMS} LevelEllipsoid)
  include_directories ("${Boost_INCLUDE_DIRS}")
  if (APPLE)
    # Suppress warnings from Boost library
    # warnings with Mac OS X and boost 1.63
    # no warnings with Linux and boost 1.60
    set (CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -Wno-unused-parameter -Wno-unused-variable -Wno-unused-local-typedef")
  endif ()
endif ()

# Loop over all the tools, specifying the source and library.
add_custom_target (develprograms)
foreach (DEVELPROGRAM ${DEVELPROGRAMS})

  add_executable (${DEVELPROGRAM} EXCLUDE_FROM_ALL ${DEVELPROGRAM}.cpp)
  add_dependencies (develprograms ${DEVELPROGRAM})
  target_link_libraries (${DEVELPROGRAM} ${PROJECT_LIBRARIES}
    ${HIGHPREC_LIBRARIES})

endforeach ()

add_executable (GeodExact EXCLUDE_FROM_ALL GeodExact.cpp
  Geodesic30.cpp GeodesicLine30.cpp
  Geodesic30.hpp GeodesicLine30.hpp)
add_dependencies (develprograms GeodExact)
target_link_libraries (GeodExact ${PROJECT_LIBRARIES} ${HIGHPREC_LIBRARIES})
set (DEVELPROGRAMS ${DEVELPROGRAMS} GeodExact)

add_executable (AreaEst EXCLUDE_FROM_ALL AreaEst.cpp)
add_dependencies (develprograms AreaEst)
target_link_libraries (AreaEst ${PROJECT_LIBRARIES} ${FFTW_LIBRARIES}
  ${HIGHPREC_LIBRARIES})
if (FFTW_FOUND)
  target_compile_definitions (AreaEst PUBLIC HAVE_FFTW=1)
  target_include_directories (AreaEst PUBLIC ${FFTW_INCLUDE_DIRS})
endif ()

set (DEVELPROGRAMS ${DEVELPROGRAMS} AreaEst)

add_executable (reformat EXCLUDE_FROM_ALL reformat.cpp)
add_dependencies (develprograms reformat)
set (DEVELPROGRAMS ${DEVELPROGRAMS} reformat)

if (MSVC OR CMAKE_CONFIGURATION_TYPES)
  # Add _d suffix for your debug versions of the tools
  set_target_properties (${DEVELPROGRAMS} PROPERTIES
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif ()

# Put all the programs into a folder in the IDE
set_property (TARGET develprograms ${DEVELPROGRAMS} PROPERTY FOLDER develop)

# Don't install develop programs
