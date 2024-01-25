cmake_minimum_required(VERSION 3.22.1)
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
        src/JsonSerializer.cpp
        src/JsonSerializer.h
)
target_compile_features(driver_ultraleap
    PUBLIC
        cxx_std_20
)
target_link_libraries(driver_ultraleap
    PRIVATE
        OpenVR::OpenVR
        LeapSDK::LeapC
        glm::glm
        nlohmann_json::nlohmann_json
)
set_target_properties(driver_ultraleap PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ultraleap/bin/win64)

# Copy the required LeapC.dll to the output folder.
add_custom_command(
        TARGET driver_ultraleap POST_BUILD
        COMMENT "Copying DLL: LeapC.dll"
        DEPENDS $<TARGET_FILE:LeapSDK::LeapC>
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:LeapSDK::LeapC> $<TARGET_FILE_DIR:driver_ultraleap>
)

# Copy the localization, resources and manifests.
add_custom_command(
        TARGET driver_ultraleap POST_BUILD
        COMMENT "Copying Resources & Manifest"
        DEPENDS ${CMAKE_SOURCE_DIR}/ultraleap
        BYPRODUCTS ${CMAKE_BINARY_DIR}/ultraleap
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/ultraleap ${CMAKE_BINARY_DIR}/ultraleap
)