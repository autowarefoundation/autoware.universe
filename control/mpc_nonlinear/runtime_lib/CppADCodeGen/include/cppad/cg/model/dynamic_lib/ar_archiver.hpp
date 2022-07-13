#ifndef CPPAD_CG_LINUX_AR_ARCHIVER_INCLUDED
#define CPPAD_CG_LINUX_AR_ARCHIVER_INCLUDED
/* --------------------------------------------------------------------------
 *  CppADCodeGen: C++ Algorithmic Differentiation with Source Code Generation:
 *    Copyright (C) 2013 Ciengis
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

class ArArchiver : public Archiver {
protected:
    std::string _arPath; // the path to the ar executable
    std::vector<std::string> _flags;
    bool _verbose;
public:

    inline ArArchiver() :
        _arPath("/usr/bin/ar"),
        _verbose(false) {
    }

    inline ArArchiver(const std::string& arPath) :
        _arPath(arPath),
        _verbose(false) {
    }

    inline virtual bool isVerbose() const {
        return _verbose;
    }

    inline virtual void setVerbose(bool verbose) {
        _verbose = verbose;
    }

    inline const std::vector<std::string>& getFlags() const {
        return _flags;
    }

    inline void setFlags(const std::vector<std::string>& flags) {
        _flags = flags;
    }

    inline virtual void create(const std::string& library,
                               const std::set<std::string>& objectFiles,
                               JobTimer* timer = nullptr) {
        // backup output format so that it can be restored
        OStreamConfigRestore coutb(std::cout);

        std::vector<std::string> args;
        args.push_back("rcs");
        args.insert(args.end(), _flags.begin(), _flags.end());
        args.push_back(library); // Output file name
        args.insert(args.end(), objectFiles.begin(), objectFiles.end());

        if (timer != nullptr) {
            timer->startingJob("'" + library + "'", JobTimer::ASSEMBLE_STATIC_LIBRARY);
        } else if (_verbose) {
            std::cout << "building library '" << library << "'" << std::endl;
        }

        system::callExecutable(_arPath, args);

        if (timer != nullptr) {
            timer->finishedJob();
        }
    }

};

} // END cg namespace
} // END CppAD namespace

#endif