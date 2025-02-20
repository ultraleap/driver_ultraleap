#include "OsUtils.h"

#include <stdexcept>

#if defined(_WIN32)
#include <ObjectArray.h>
#endif

// Sets the name of the current thread.
auto OsUtils::SetThreadName(std::thread& thread, const std::string_view& name) -> void {
    // Set the thread name (This has to utilize a platform specific method).
#if defined(_WIN32)
    const auto narrow_name = std::string{name};
    const auto wide_length = MultiByteToWideChar(CP_UTF8, 0, narrow_name.c_str(), -1, nullptr, 0);
    auto wide_name = std::wstring(wide_length, 0);
    if (MultiByteToWideChar(CP_UTF8, 0, narrow_name.c_str(), -1, wide_name.data(), wide_length) == 0) {
        throw std::runtime_error("Failed to convert thread name");
    }
    if (FAILED(SetThreadDescription(thread.native_handle(), wide_name.c_str()))) {
        throw std::runtime_error("Failed to set thread name");
    }
#elif defined(__linux__)
    if (pthread_setname_np(thread.native_handle(), std::string{name}.c_str()) != 0) {
        throw std::runtime_error("Failed to set thread name");
    };
#else
#error "OsUtils::SetThreadName() not implemented for current platform"
#endif
}