// Copyright 2022 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef COMMUNICATION_DELAY_COMPENSATOR__DELAY_COMPENSATION_UTILS_HPP
#define COMMUNICATION_DELAY_COMPENSATOR__DELAY_COMPENSATION_UTILS_HPP

#include <utility>
#include <type_traits>
#include <limits>
#include <vector>
#include <filesystem>
#include <fstream>
#include "visibility_control.hpp"

#define GET_VARIABLE_NAME(Variable) (#Variable)
auto constexpr EPS = std::numeric_limits<double>::epsilon();

namespace fs = std::filesystem;


/**
 * @brief Implements strong type definition for defining different class constructor behaviors with the same kind of
 * parameter pack.
 * */

template<typename T, typename Parameter>
class StrongTypeDef {
public:
    explicit StrongTypeDef(T const &value) : value_(value) {
    }

    explicit StrongTypeDef(T &&value) : value_(std::move(value)) {
    }

    T &get() {
        return value_;
    }

    T const &get() const {
        return value_;
    }

private:
    T value_;
};

/**
 * @brief Fetching the underlying type from strongly typed Enum class.
 * */
template<typename E>
constexpr auto toUType(E e) noexcept {
    return static_cast<std::underlying_type_t<E>>(e);
}


// ************* WRITE TO PATH **************************

template<typename T>
void writeToFile(const fs::path &outputPath, T var, std::string varname) {
    /**
     * @brief writes the given variable into the folder in txt format
     * @param outputPath path of the folder the txt file goes into
     * @param var   the variable to be written into the file
     * @param varname name of the txt file
     *
     * */

    if (not fs::exists(outputPath) and not fs::create_directories(outputPath)) {
        throw std::runtime_error("Could not create output directory!");
    }

    varname += ".txt";
    std::ofstream f(outputPath / varname);

    f << var;

}

template<typename T>
void writeToFile(const fs::path &outputPath, std::vector<T> var, std::string varname) {
    /**
     * @brief writes the given variable into the folder in txt format
     * @param outputPath path of the folder the txt file goes into
     * @param var   the variable to be written into the file
     * @param varname name of the txt file
     *
     * */

    if (not fs::exists(outputPath) and not fs::create_directories(outputPath)) {
        throw std::runtime_error("Could not create output directory!");
    }

    varname += ".txt";
    std::ofstream f(outputPath / varname);

    for (auto &&x: var) {
        f << x << " ";
    }

}


#endif //COMMUNICATION_DELAY_COMPENSATOR__DELAY_COMPENSATION_UTILS_HPP
