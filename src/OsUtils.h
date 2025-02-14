#pragma once

// ReSharper disable once CppUnusedIncludeDirective
#include <string_view>
#include <thread>

class OsUtils {
  public:
    OsUtils() = delete;
    static auto SetThreadName(std::thread& thread, const std::string_view& name) -> void;
};
