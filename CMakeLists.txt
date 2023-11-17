cmake_minimum_required(VERSION 3.22.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

project(driver_ultraleap)

set(CMAKE_CXX_STANDARD 23)

set(LeapSDK_DIR ${CMAKE_SOURCE_DIR}/3rdparty/LeapSDK/lib/cmake/LeapSDK)
set(OpenVR_ROOT ${CMAKE_SOURCE_DIR}/3rdparty/OpenVR)
find_package(OpenVR REQUIRED)
find_package(LeapSDK 5 REQUIRED)

add_library(driver_ultraleap SHARED
        src/HmdDriverFactory.cpp
        src/LeapDeviceProvider.cpp
        src/LeapDeviceProvider.h
        src/LeapDeviceDriver.cpp
        src/LeapDeviceDriver.h
        src/LeapHandDriver.cpp
        src/LeapHandDriver.h
        src/LeapDevice.cpp
        src/LeapDevice.h
        src/OsUtils.cpp
        src/OsUtils.h
        src/VrUtils.h
        src/VrLogging.h
)
target_link_libraries(driver_ultraleap
        PUBLIC OpenVR::OpenVR
        PRIVATE LeapSDK::LeapC)
target_include_directories(driver_ultraleap
        PRIVATE ${OpenVR_INCLUDE_DIR}/../samples/drivers/utils/vrmath/
)
set_target_properties(driver_ultraleap PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/ultraleap/bin/win64)

# Copy the required LeapC.dll to the output folder.
add_custom_command(
        TARGET driver_ultraleap POST_BUILD
        COMMENT "Copying DLL: LeapC.dll"
        DEPENDS $<TARGET_FILE:LeapSDK::LeapC>
        #BYPRODUCTS $<TARGET_FILE_DIR:driver_ultraleap>/$<TARGET_FILE_NAME:LeapSDK::LeapC>
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:LeapSDK::LeapC> $<TARGET_FILE_DIR:driver_ultraleap>)

# Copy the localization, resources and manifests.
add_custom_command(
        TARGET driver_ultraleap POST_BUILD
        COMMENT "Copying Driver Manifest"
        DEPENDS ${CMAKE_SOURCE_DIR}/driver.vrdrivermanifest
        BYPRODUCTS ${CMAKE_BINARY_DIR}/ultraleap/driver.vrdrivermanifest
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_SOURCE_DIR}/driver.vrdrivermanifest ${CMAKE_BINARY_DIR}/ultraleap/driver.vrdrivermanifest)
add_custom_command(
        TARGET driver_ultraleap POST_BUILD
        COMMENT "Copying Resources"
        DEPENDS ${CMAKE_SOURCE_DIR}/resources
        BYPRODUCTS ${CMAKE_BINARY_DIR}/ultraleap/resources
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/resources ${CMAKE_BINARY_DIR}/ultraleap/resources)