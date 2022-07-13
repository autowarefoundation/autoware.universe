#ifndef CPPAD_CG_SYSTEM_INCLUDED
#define CPPAD_CG_SYSTEM_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2012 Ciengis
 *
 *  CppADCodeGen is distributed under multiple licenses:
 *
 *   - Eclipse Public License Version 1.0 (EPL1), and
 *   - GNU General Public License Version 3 (GPL3).
 *
 *  EPL1 terms and conditions can be found in the file "epl-v10.txt", while
 *  terms and conditions for the GPL3 can be found in the file "gpl3.txt".
 * ----------------------------------------------------------------------------
 * Author: Joao Leal
 */

namespace CppAD {
namespace cg {

/**
 * System dependent functions
 */
namespace system {

template<class T = int >
class SystemInfo {
public:
    static const std::string DYNAMIC_LIB_EXTENSION;
    static const std::string STATIC_LIB_EXTENSION;
};

inline std::string getWorkingDirectory();

/**
 * creates a new folder (system dependent)
 * 
 * @param folder the path to the folder
 * @throws CGException on failure to create the folder
 */
inline void createFolder(const std::string& folder);

/**
 * Creates a new path (system dependent)
 * 
 * @param baseFolder the path to the base folder
 * @param file the file or folder name inside the base folder
 * @return the new path
 */
inline std::string createPath(const std::string& baseFolder,
                              const std::string& file);

/**
 * Creates a new path (system dependent)
 *
 * @param folders the names of the folders in the path to the file
 * @param file the file or folder name
 * @return the new path
 */
inline std::string createPath(std::initializer_list<std::string> folders,
                              const std::string& file);

/**
 * Escapes a file or folder path (system dependent)
 * 
 * @param path the file/folder path
 * @return the escaped file/folder path
 */
inline std::string escapePath(const std::string& path);

inline std::string filenameFromPath(const std::string& path);

inline std::string directoryFromPath(const std::string& path);

/**
 * Determines if a path is absolute.
 *
 * @param path the path
 * @return true if it is absolute, false if it is relative
 */
inline bool isAbsolutePath(const std::string& path);

/**
 * Checks if a path exists and is a directory
 *
 * @param path the path
 * @return true if the path exists and it is a directory
 */
inline bool isDirectory(const std::string& path);

/**
 * Checks if a path exists and is a file
 *
 * @param path the path
 * @return true if the path exists and it is a file
 */
inline bool isFile(const std::string& path);

/**
 * Calls an external executable (system dependent).
 * In the case of an error during execution an exception will be thrown.
 * 
 * @param executable the executable path
 * @param args the command line arguments to the executable
 * @param stdOutErrMessage standard output and standard error message
 *                         from the executable
 * @param stdInMessage information to pass as standard input to the executable
 * @throws CGException on failure to call the executable
 */
inline void callExecutable(const std::string& executable,
                           const std::vector<std::string>& args,
                           std::string* stdOutErrMessage = nullptr,
                           const std::string* stdInMessage = nullptr);

}

} // END cg namespace
} // END CppAD namespace

#endif