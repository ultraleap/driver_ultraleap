#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "LeapSDK::LeapC" for configuration "RelWithDebInfo"
set_property(TARGET LeapSDK::LeapC APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
list(APPEND _IMPORT_CHECK_TARGETS LeapSDK::LeapC )

if(WIN32)
    set_target_properties(LeapSDK::LeapC PROPERTIES
            IMPORTED_IMPLIB_RELWITHDEBINFO "${_IMPORT_PREFIX}/LeapSDK/lib/x64/LeapC.lib"
            IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/LeapSDK/lib/x64/LeapC.dll"
            )
    list(APPEND _IMPORT_CHECK_FILES_FOR_LeapSDK::LeapC "${_IMPORT_PREFIX}/LeapSDK/lib/x64/LeapC.lib" "${_IMPORT_PREFIX}/LeapSDK/lib/x64/LeapC.dll" )
elseif(UNIX)
    set_target_properties(LeapSDK::LeapC PROPERTIES
            IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/LeapSDK/lib/x64/libLeapC.so.5"
            IMPORTED_SONAME_RELWITHDEBINFO "libLeapC.so.5"
            )
    list(APPEND _IMPORT_CHECK_FILES_FOR_LeapSDK::LeapC "${_IMPORT_PREFIX}/LeapSDK/lib/x64/libLeapC.so.5")
endif()

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
