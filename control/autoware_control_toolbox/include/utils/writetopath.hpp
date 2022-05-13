// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#define GET_VARIABLE_NAME(Variable) (#Variable)

#include <experimental/filesystem> // fmt package
#include <fstream>
#include <string>
#include <vector>

// for creating an output file
namespace fs = std::experimental::filesystem;

fs::path getOutputPath();

template<typename T>
void writeToFile(const fs::path &outputPath, T var, std::string varname)
{
    /**
     * @brief writes the given variable into the folder in txt format
     * @param outputPath path of the folder the txt file goes into
     * @param var   the variable to be written into the file
     * @param varname name of the txt file
     *
     * */

    if (not fs::exists(outputPath) && !fs::create_directories(outputPath))
    {
        throw std::runtime_error("Could not create output directory!");
    }

    varname += ".txt";
    std::ofstream f(outputPath / varname);

    f << var;

}

template<typename T>
void writeToFile(const fs::path &outputPath, std::vector <T> var, std::string varname)
{
    /**
     * @brief writes the given variable into the folder in txt format
     * @param outputPath path of the folder the txt file goes into
     * @param var   the variable to be written into the file
     * @param varname name of the txt file
     *
     * */

    if (not fs::exists(outputPath) and not fs::create_directories(outputPath))
    {
        throw std::runtime_error("Could not create output directory!");
    }

    varname += ".txt";
    std::ofstream f(outputPath / varname);

    for (auto &&x: var)
    {
        f << x << " ";
    }

}
