INCLUDE(FindPackageHandleStandardArgs)
INCLUDE(HandleLibraryTypes)

SET(LCM_IncludeSearchPaths
  /usr/include/
  /usr/local/include/
  /opt/local/include
)

SET(LCM_LibrarySearchPaths
  /usr/lib/
  /usr/local/lib/
  /opt/local/lib/
)

FIND_PATH(LCM_INCLUDE_DIR lcm/lcm.h
  PATHS ${LCM_IncludeSearchPaths}
)
FIND_LIBRARY(LCM_LIBRARY_OPTIMIZED
  NAMES lcm
  PATHS ${LCM_LibrarySearchPaths}
)

# Handle the REQUIRED argument and set the <UPPERCASED_NAME>_FOUND variable
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LCM "Could NOT find LCM library (LCM)"
  LCM_LIBRARY_OPTIMIZED
  LCM_INCLUDE_DIR
)

# Collect optimized and debug libraries
HANDLE_LIBRARY_TYPES(LCM)

MARK_AS_ADVANCED(
  LCM_INCLUDE_DIR
  LCM_LIBRARY_OPTIMIZED
)
