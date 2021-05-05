#pragma once

#include <stdexcept>
#include <spdlog/fmt/fmt.h>

/*** spdlog commands ***
Write output:
spdlog::critical("Message {}", example_variable)
spdlog::error("Message {}", example_variable)
spdlog::warn("Message {}", example_variable)
spdlog::info("Message {}", example_variable)
spdlog::debug("Message {}", example_variable)

Change debug level:
spdlog::set_level(spdlog::level::info); // Set global log level to info

Create named output:
const auto console = spdlog::stdout_color_st("OutputName");
console->info("Test message")
*/

namespace formatting {

using default_exception_type = std::runtime_error;

inline const char* basename(const char* path)
{
    // C-style basename needs a char * as input, no choice but to const_cast it
    return ::basename(const_cast<char*>(path));
}

template<typename Expression, typename... Args>
inline constexpr void require(bool condition, const char*, const char* file, int line, Expression&&, Args&&... args)
{
    if (not condition) {
        throw default_exception_type(fmt::format("{}:{}: {}", basename(file), line, fmt::format(std::forward<Args>(args)...)));
    }
}

template<typename Expression>
inline constexpr void require(bool condition, const char* expr, const char* file, int line, Expression&&)
{
    if (not condition) {
        throw default_exception_type(fmt::format("{}:{}: '{}' is false!", basename(file), line, expr));
    }
}

template<typename First, typename... Rest>
inline constexpr First arg0(First&& first, Rest&&...)
{
    return std::forward<First>(first);
}

template<typename Exception = default_exception_type, typename... Args>
inline constexpr void require(bool condition, Args&&... args)
{
    if (not condition) {
        throw default_exception_type(fmt::format(std::forward<Args>(args)...));
    }
}

template<typename Exception = default_exception_type>
inline constexpr void require(bool condition)
{
    if (not condition) {
        throw Exception("Requirement not met!");
    }
}

} // namespace formatting

#define REQUIRE(...) formatting::require(static_cast<bool>(formatting::arg0(__VA_ARGS__)), #__VA_ARGS__, __FILE__, __LINE__, __VA_ARGS__)
