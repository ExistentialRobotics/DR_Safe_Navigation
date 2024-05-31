#pragma once

#include "fmt.hpp"
#include "progress_bar.hpp"

#include <mutex>

namespace erl::common {
    struct Logging {

        static std::string
        GetDateStr() {
            return fmt::format("{:%Y-%m-%d}", fmt::localtime(std::time(nullptr)));
        }

        static std::string
        GetTimeStr() {
            return fmt::format("{:%X}", fmt::localtime(std::time(nullptr)));
        }

        static std::string
        GetDateTimeStr() {
            return fmt::format("{:%Y-%m-%d %X}", fmt::localtime(std::time(nullptr)));
        }

        static std::string
        GetTimeStamp() {
            return fmt::format("{:%Y%m%d-%H%M%S}", fmt::localtime(std::time(nullptr)));
        }

        template<typename... Args>
        static void
        Info(Args... args) {
            // https://fmt.dev/latest/syntax.html
            std::string msg = fmt::format(fmt::fg(fmt::color::deep_sky_blue) | fmt::emphasis::bold, "[{:%X}][INFO]: ", fmt::localtime(std::time(nullptr)));
            fmt::format_to(std::back_inserter(msg), std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { msg += "\n"; }
            ProgressBar::Write(msg);
        }

        template<typename... Args>
        static void
        Debug(Args... args) {
            std::string msg = fmt::format(fmt::fg(fmt::color::orange) | fmt::emphasis::bold, "[{:%X}][DEBUG]: ", fmt::localtime(std::time(nullptr)));
            fmt::format_to(std::back_inserter(msg), std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { msg += "\n"; }
            ProgressBar::Write(msg);
        }

        template<typename... Args>
        static void
        Warn(Args... args) {
            std::string msg = fmt::format(fmt::fg(fmt::color::orange_red) | fmt::emphasis::bold, "[{:%X}][WARN]: ", fmt::localtime(std::time(nullptr)));
            fmt::format_to(std::back_inserter(msg), std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { msg += "\n"; }
            ProgressBar::Write(msg);
        }

        template<typename... Args>
        static void
        Error(Args... args) {
            std::string msg = fmt::format(fmt::fg(fmt::color::red) | fmt::emphasis::bold, "[{:%X}][ERROR]: ", fmt::localtime(std::time(nullptr)));
            fmt::format_to(std::back_inserter(msg), std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { msg += "\n"; }
            ProgressBar::Write(msg);
        }

        template<typename... Args>
        static void
        Fatal(Args... args) {
            std::string msg = fmt::format(fmt::fg(fmt::color::dark_red) | fmt::emphasis::bold, "[{:%X}][FATAL]: ", fmt::localtime(std::time(nullptr)));
            fmt::format_to(std::back_inserter(msg), std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { msg += "\n"; }
            ProgressBar::Write(msg);
        }

        template<typename... Args>
        static void
        Success(Args... args) {
            std::string msg = fmt::format(fmt::fg(fmt::color::spring_green) | fmt::emphasis::bold, "[{:%X}][SUCCESS]: ", fmt::localtime(std::time(nullptr)));
            fmt::format_to(std::back_inserter(msg), std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { msg += "\n"; }
            ProgressBar::Write(msg);
        }

        template<typename... Args>
        static std::string
        Failure(Args... args) {
            const std::string msg = fmt::format(fmt::fg(fmt::color::red) | fmt::emphasis::bold, "[{:%X}][FAILURE]: ", fmt::localtime(std::time(nullptr)));
            std::string failure_msg = fmt::format(std::forward<Args>(args)...);
            if (ProgressBar::GetNumBars() == 0) { failure_msg += "\n"; }
            ProgressBar::Write(msg + failure_msg);
            return failure_msg;
        }

        static void
        Write(const std::string& msg) {
            ProgressBar::Write(msg);
        }
    };
}  // namespace erl::common

static std::mutex g_print_mutex;

#if defined(ERL_ROS_VERSION_1) || defined(ERL_ROS_VERSION_2)
    #include <ros/assert.h>
    #include <ros/console.h>
    #define ERL_FATAL(...)                ROS_FATAL(fmt::format(__VA_ARGS__).c_str())
    #define ERL_ERROR(...)                ROS_ERROR(fmt::format(__VA_ARGS__).c_str())
    #define ERL_WARN(...)                 ROS_WARN(fmt::format(__VA_ARGS__).c_str())
    #define ERL_WARN_ONCE(...)            ROS_WARN_ONCE(fmt::format(__VA_ARGS__).c_str())
    #define ERL_WARN_COND(condition, ...) ROS_WARN_COND(condition, fmt::format(__VA_ARGS__).c_str())
    #define ERL_INFO(...)                 ROS_INFO(fmt::format(__VA_ARGS__).c_str())
    #define ERL_DEBUG(...)                ROS_DEBUG(fmt::format(__VA_ARGS__).c_str())
    #ifdef ROS_ASSERT_ENABLED
        #define ERL_ASSERT(expr) ROS_ASSERT(expr)
        #define ERL_ASSERTM(expr, ...) \
            do { ROS_ASSERT_MSG(expr, fmt::format(__VA_ARGS__).c_str()); } while (false)
    #endif
#else
    #define ERL_FATAL(...)                                                                          \
        do {                                                                                        \
            g_print_mutex.lock();                                                                   \
            erl::common::Logging::Fatal("{}:{}: {}", __FILE__, __LINE__, fmt::format(__VA_ARGS__)); \
            g_print_mutex.unlock();                                                                 \
            exit(1);                                                                                \
        } while (false)

    #define ERL_ERROR(...)                                                                          \
        do {                                                                                        \
            g_print_mutex.lock();                                                                   \
            erl::common::Logging::Error("{}:{}: {}", __FILE__, __LINE__, fmt::format(__VA_ARGS__)); \
            g_print_mutex.unlock();                                                                 \
        } while (false)

    #define ERL_WARN(...)                                                                          \
        do {                                                                                       \
            g_print_mutex.lock();                                                                  \
            erl::common::Logging::Warn("{}:{}: {}", __FILE__, __LINE__, fmt::format(__VA_ARGS__)); \
            g_print_mutex.unlock();                                                                \
        } while (false)

    #define ERL_WARN_ONCE(...)          \
        do {                            \
            static bool warned = false; \
            if (!warned) {              \
                warned = true;          \
                ERL_WARN(__VA_ARGS__);  \
            }                           \
        } while (false)

    #define ERL_WARN_COND(condition, ...)             \
        do {                                          \
            if (condition) { ERL_WARN(__VA_ARGS__); } \
        } while (false)

    #define ERL_INFO(...)                                                                          \
        do {                                                                                       \
            g_print_mutex.lock();                                                                  \
            erl::common::Logging::Info("{}:{}: {}", __FILE__, __LINE__, fmt::format(__VA_ARGS__)); \
            g_print_mutex.unlock();                                                                \
        } while (false)

    #define ERL_INFO_ONCE(...)          \
        do {                            \
            static bool infoed = false; \
            if (!infoed) {              \
                infoed = true;          \
                ERL_INFO(__VA_ARGS__);  \
            }                           \
        } while (false)

    #ifndef NDEBUG
        #define ERL_DEBUG(...)                                                                          \
            do {                                                                                        \
                g_print_mutex.lock();                                                                   \
                erl::common::Logging::Debug("{}:{}: {}", __FILE__, __LINE__, fmt::format(__VA_ARGS__)); \
                g_print_mutex.unlock();                                                                 \
            } while (false)
        #define ERL_DEBUG_ASSERT(expr, ...) ERL_ASSERTM(expr, __VA_ARGS__)
    #else
        #define ERL_DEBUG(...)              ((void) 0)
        #define ERL_DEBUG_ASSERT(expr, ...) (void) 0
    #endif
#endif

#define ERL_WARN_ONCE_COND(condition, ...) \
    do {                                   \
        static bool warned = false;        \
        if (!warned && (condition)) {      \
            warned = true;                 \
            ERL_WARN(__VA_ARGS__);         \
        }                                  \
    } while (false)

#ifndef ERL_ASSERTM
    #define ERL_ASSERTM(expr, ...)                                                                                                                           \
        do {                                                                                                                                                 \
            if (!(expr)) {                                                                                                                                   \
                g_print_mutex.lock();                                                                                                                        \
                std::string failure_msg = erl::common::Logging::Failure("assertion ({}) at {}:{}: {}", #expr, __FILE__, __LINE__, fmt::format(__VA_ARGS__)); \
                g_print_mutex.unlock();                                                                                                                      \
                throw std::runtime_error(failure_msg);                                                                                                       \
            }                                                                                                                                                \
        } while (false)
#endif

#ifndef ERL_ASSERT
    #define ERL_ASSERT(expr) ERL_ASSERTM(expr, "Assertion {} failed.", #expr)
#endif

#ifndef NDEBUG
    #define ERL_DEBUG_ASSERT(expr, ...)              ERL_ASSERTM(expr, __VA_ARGS__)
    #define ERL_DEBUG_WARN_COND(condition, ...)      ERL_WARN_COND(condition, __VA_ARGS__)
    #define ERL_DEBUG_WARN_ONCE_COND(condition, ...) ERL_WARN_ONCE_COND(condition, __VA_ARGS__)
#else
    #define ERL_DEBUG_ASSERT(expr, ...)              (void) 0
    #define ERL_DEBUG_WARN_COND(condition, ...)      (void) 0
    #define ERL_DEBUG_WARN_ONCE_COND(condition, ...) (void) 0
#endif
