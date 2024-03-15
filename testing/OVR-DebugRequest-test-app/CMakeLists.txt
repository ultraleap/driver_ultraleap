cmake_minimum_required(VERSION 3.27)
project(OVR_DebugRequest_test_app)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

set(3rdparty_ROOT ${CMAKE_SOURCE_DIR}/../../3rdparty)
set(OpenVR_ROOT ${3rdparty_ROOT}/OpenVR)
find_package(OpenVR REQUIRED)
add_subdirectory(${3rdparty_ROOT}/json ${3rdparty_ROOT}/json EXCLUDE_FROM_ALL)

add_executable(OVR_DebugRequest_test_app
        src/main.cpp)

target_compile_features(OVR_DebugRequest_test_app
        PUBLIC
        cxx_std_20
)

target_link_libraries(OVR_DebugRequest_test_app
        PRIVATE
        OpenVR::OpenVR
        nlohmann_json::nlohmann_json
)

# Copy the required dynamically linked lib
add_custom_command(
        TARGET OVR_DebugRequest_test_app POST_BUILD
        COMMENT "Copying openvr_api.dll"
        DEPENDS ${OpenVR_DLL_DIR}
        BYPRODUCTS ${CMAKE_BINARY_DIR}/openvr_api.dll
        COMMAND ${CMAKE_COMMAND} -E copy ${OpenVR_DLL_DIR}/openvr_api.dll ${CMAKE_BINARY_DIR}/openvr_api.dll
)