//
// Created by ali on 14/8/19.
//
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
void writeToFile(const fs::path& outputPath, T var, std::string varname)
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
void writeToFile(const fs::path& outputPath, std::vector<T> var, std::string varname)
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

	for (auto&& x: var)
	{
		f << x << " ";
	}

}
