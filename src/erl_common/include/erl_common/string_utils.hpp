#pragma once

#include <cxxabi.h>

#include <memory>
#include <sstream>
#include <typeinfo>

#ifdef __GNUG__
inline std::string
demangle(const char *name) {
    int status = -4;  // some arbitrary value to eliminate the compiler warning
    std::unique_ptr<char, void (*)(void *)> res{abi::__cxa_demangle(name, nullptr, nullptr, &status), std::free};
    return (status == 0) ? res.get() : name;
}
#else
inline std::string
demangle(const char *name) {
    return name;
}
#endif

template<typename T>
std::string
type_name() {
    return demangle(typeid(T).name());
}

namespace erl::common {

    template<typename... Args>
    std::string
    AsString(Args... args) {
        std::stringstream ss;
        (ss << ... << args);  // https://en.cppreference.com/w/cpp/language/fold
        return ss.str();
    }
}  // namespace erl::common
