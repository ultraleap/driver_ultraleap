cmake_minimum_required(VERSION 3.26)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(driver_ultraleap)

set(LeapSDK_DIR ${CMAKE_SOURCE_DIR}/3rdparty/LeapSDK/lib/cmake/LeapSDK)
set(OpenVR_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/OpenVR)
set(GLM_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/GLM)

find_package(OpenVR REQUIRED)
find_package(LeapSDK 5 REQUIRED)
add_subdirectory(3rdparty/json EXCLUDE_FROM_ALL)

# Add GLM (avoiding an annoying target it adds by creating a dummy one with the same name).
add_custom_target(uninstall)
add_subdirectory(${GLM_ROOT})

option(BUILD_TEST_APPS "Build test applications" ON)

add_library(driver_ultraleap SHARED
        src/LeapDriverFactory.cpp
        src/LeapDeviceProvider.cpp
        src/LeapDeviceProvider.h
        src/LeapDeviceDriver.cpp
        src/LeapDeviceDriver.h
        src/LeapHandDriver.cpp
        src/LeapHandDriver.h
        src/LeapElbowDriver.cpp
        src/LeapElbowDriver.h
        src/LeapDevice.cpp
        src/LeapDevice.h
        src/OsUtils.cpp
        src/OsUtils.h
        src/VrUtils.h
        src/VrLogging.h
        src/VrMaths.h
        src/VrHand.cpp
        src/VrHand.h
        src/LeapDriverSettings.cpp
        src/LeapDriverSettings.h
        src/RequestDeserializer.cpp
        src/RequestDeserializer.h
        src/LeapTrackedDriver.cpp
        src/LeapTrackedDriver.h
)
target_compile_features(driver_ultraleap PUBLIC cxx_std_20)
target_link_libraries(driver_ultraleap
    PRIVATE
        OpenVR::Driver
        LeapSDK::LeapC
        glm::glm
        nlohmann_json::nlohmann_json
)
set_target_properties(driver_ultraleap PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ultraleap/bin/win64)

# Copy the required DLL dependencies.
add_custom_command(
        TARGET driver_ultraleap POST_BUILD
        COMMENT "Copying DLLs"
        COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:driver_ultraleap> $<TARGET_RUNTIME_DLLS:driver_ultraleap>
        COMMAND_EXPAND_LISTS
)

add_custom_target(driver_ultraleap_resources)
add_custom_command(
        TARGET driver_ultraleap_resources POST_BUILD
        COMMENT "Copying Resources & Manifest"
        COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different ${CMAKE_SOURCE_DIR}/ultraleap ${CMAKE_BINARY_DIR}/ultraleap
)
add_dependencies(driver_ultraleap driver_ultraleap_resources)

# Build test applications if requested.
if (BUILD_TEST_APPS)
    add_executable(test_debug_request test/TestDebugRequest.cpp)
    target_compile_features(test_debug_request PUBLIC cxx_std_20)
    target_link_libraries(test_debug_request
        PRIVATE
            OpenVR::API
            nlohmann_json::nlohmann_json
    )

    # Copy the required DLL dependencies.
    add_custom_command(
        TARGET test_debug_request POST_BUILD
        COMMENT "Copying DLLs"
        COMMAND ${CMAKE_COMMAND} -E copy -t $<TARGET_FILE_DIR:test_debug_request> $<TARGET_RUNTIME_DLLS:test_debug_request>
        COMMAND_EXPAND_LISTS
    )
    add_dependencies(test_debug_request driver_ultraleap)
endif()
