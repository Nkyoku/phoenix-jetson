#pragma once

#include <string>
#include <string_view>

static std::string constructName(std::string_view base_name, std::string child_name) {
    std::string result(base_name);
    if (base_name.back() != '/') {
        result += '/';
    }
    result += child_name;
    return result;
}
