#----------------------------------------------------------------
# Generated CMake target import file for configuration "relwithdebinfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LeapSDK::LeapC" for configuration "relwithdebinfo"
set_property(TARGET LeapSDK::LeapC APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(LeapSDK::LeapC PROPERTIES
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/LeapSDK/lib/arm64-v8a/libLeapC.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libLeapC.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS LeapSDK::LeapC )
list(APPEND _IMPORT_CHECK_FILES_FOR_LeapSDK::LeapC "${_IMPORT_PREFIX}/LeapSDK/lib/arm64-v8a/libLeapC.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
