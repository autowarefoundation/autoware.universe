from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils.old_build_ext import old_build_ext as build_ext
import numpy, os, platform, sys
from os.path import join as pjoin


# Obtain the numpy include directory.  This logic works across numpy versions.
try:
    numpy_include = numpy.get_include()
except AttributeError:
    numpy_include = numpy.get_numpy_include()

def check_for_flag(flag_str, truemsg=False, falsemsg=False):
    if flag_str in os.environ:
        enabled = (os.environ[flag_str].lower() == "on")
    else:
        enabled = False

    if enabled and not truemsg == False:
        print(truemsg)
    elif not enabled and not falsemsg == False:
        print(falsemsg)
        print("   $ sudo "+flag_str+"=ON python setup.py install")
    return enabled

use_cuda = check_for_flag("WITH_CUDA", \
    "Compiling with CUDA support", \
    "Compiling without CUDA support. To enable CUDA use:")
trace    = check_for_flag("TRACE", \
    "Compiling with trace enabled for Bresenham's Line", \
    "Compiling without trace enabled for Bresenham's Line")

print()
print("--------------")
print()

# support for compiling in clang
if platform.system().lower() == "darwin":
    os.environ["MACOSX_DEPLOYMENT_TARGET"] = platform.mac_ver()[0]
    os.environ["CC"] = "c++"

def find_in_path(name, path):
    "Find a file in a search path"
    #adapted fom http://code.activestate.com/recipes/52224-find-a-file-given-a-search-path/
    for dir in path.split(os.pathsep):
        binpath = pjoin(dir, name)
        if os.path.exists(binpath):
            return os.path.abspath(binpath)
    return None

# export CUDAHOME=/usr/local/cuda
def locate_cuda():
    """Locate the CUDA environment on the system

    Returns a dict with keys 'home', 'nvcc', 'include', and 'lib64'
    and values giving the absolute path to each directory.

    Starts by looking for the CUDAHOME env variable. If not found, everything
    is based on finding 'nvcc' in the PATH.
    """
    # print os.environ
    # first check if the CUDAHOME env variable is in use
    if os.path.isdir("/usr/local/cuda-11.4"):
        home = "/usr/local/cuda-11.4"
        print('CUDA in: ' + home)
        nvcc = pjoin(home, 'bin', 'nvcc')
    elif os.path.isdir("/usr/local/cuda"):
        home = "/usr/local/cuda"
        print('CUDA in: ' + home)
        nvcc = pjoin(home, 'bin', 'nvcc')
    elif 'CUDAHOME' in os.environ:
        home = os.environ['CUDAHOME']
        print('CUDA in: ' + home)
        nvcc = pjoin(home, 'bin', 'nvcc')
    else:
        # otherwise, search the PATH for NVCC
        nvcc = find_in_path('nvcc', os.environ['PATH'])
        if nvcc is None:
            raise EnvironmentError('The nvcc binary could not be '
                'located in your $PATH. Either add it to your path, or set $CUDAHOME')
        home = os.path.dirname(os.path.dirname(nvcc))

    cudaconfig = {'home':home, 'nvcc':nvcc,
                  'include': pjoin(home, 'include'),
                  'lib64': pjoin(home, 'lib64')}
    for k, v in cudaconfig.items():
        if not os.path.exists(v):
            raise EnvironmentError('The CUDA %s path could not be located in %s' % (k, v))
    print('CUDA config: ', cudaconfig)
    return cudaconfig


##################### Configuration ############################


# compiler_flags = ["-w","-std=c++11", "-march=native", "-ffast-math", "-fno-math-errno"]
compiler_flags = ["-w","-std=c++11", "-march=native", "-ffast-math", "-fno-math-errno", "-O3"]
nvcc_flags = ['-arch=sm_62', '--ptxas-options=-v', '-c', '--compiler-options', "'-fPIC'", "-w","-std=c++11"]
include_dirs = ["../", numpy_include]
depends = ["../includes/*.h"]
sources = ["RangeLibc.pyx","../vendor/lodepng/lodepng.cpp"]

CHUNK_SIZE = "262144"
NUM_THREADS = "256"

if use_cuda:
    compiler_flags.append("-DUSE_CUDA=1");        nvcc_flags.append("-DUSE_CUDA=1")
    compiler_flags.append("-DCHUNK_SIZE="+CHUNK_SIZE); nvcc_flags.append("-DCHUNK_SIZE="+CHUNK_SIZE)
    compiler_flags.append("-DNUM_THREADS="+NUM_THREADS);   nvcc_flags.append("-DNUM_THREADS="+NUM_THREADS)
    compiler_flags.append("-DCMAKE_C_COMPILER=$(which gcc-9)")
    compiler_flags.append("-DCMAKE_CXX_COMPILER=$(which g++-9)")
    nvcc_flags.append("-DCMAKE_C_COMPILER=$(which gcc-9)")
    nvcc_flags.append("-DCMAKE_CXX_COMPILER=$(which g++-9)")

    CUDA = locate_cuda()
    include_dirs.append(CUDA['include'])
    sources.append("../includes/kernels.cu")

if trace:
    compiler_flags.append("-D_MAKE_TRACE_MAP=1")
    compiler_flags.append("-DCMAKE_C_COMPILER=$(which gcc-9)")
    compiler_flags.append("-DCMAKE_CXX_COMPILER=$(which g++-9)")


##################################################################

def customize_compiler_for_nvcc(self):
    """inject deep into distutils to customize how the dispatch
    to gcc/nvcc works.

    If you subclass UnixCCompiler, it's not trivial to get your subclass
    injected in, and still have the right customizations (i.e.
    distutils.sysconfig.customize_compiler) run on it. So instead of going
    the OO route, I have this. Note, it's kindof like a wierd functional
    subclassing going on."""

    # tell the compiler it can processes .cu
    self.src_extensions.append('.cu')

    # save references to the default compiler_so and _comple methods
    default_compiler_so = self.compiler_so
    super = self._compile

    # now redefine the _compile method. This gets executed for each
    # object but distutils doesn't have the ability to change compilers
    # based on source extension: we add it.
    def _compile(obj, src, ext, cc_args, extra_postargs, pp_opts):
        if os.path.splitext(src)[1] == '.cu':
            # use the cuda for .cu files
            self.set_executable('compiler_so', CUDA['nvcc'])
            # use only a subset of the extra_postargs, which are 1-1 translated
            # from the extra_compile_args in the Extension class
            postargs = extra_postargs['nvcc']
        else:
            postargs = extra_postargs['gcc']
        # postargs = extra_postargs#['gcc']

        super(obj, src, ext, cc_args, postargs, pp_opts)
        # reset the default compiler_so, which we might have changed for cuda
        self.compiler_so = default_compiler_so

    # inject our redefined _compile method into the class
    self._compile = _compile

# run the customize_compiler
class custom_build_ext(build_ext):
    def build_extensions(self):
        customize_compiler_for_nvcc(self.compiler)
        build_ext.build_extensions(self)

if use_cuda:
    ext = Extension("range_libc", sources, 
                    extra_compile_args = {'gcc': compiler_flags, 'nvcc': nvcc_flags},
                    extra_link_args = ["-std=c++11"],
                    include_dirs = include_dirs,
                    library_dirs=[CUDA['lib64']],
                    libraries=['cudart'],
                    runtime_library_dirs=[CUDA['lib64']],
                    depends=depends,
                    language="c++",)
    setup(name='range_libc',
        author='Corey Walsh',
        version='0.1',
        ext_modules = [ext],
        # inject our custom trigger
        cmdclass={'build_ext': custom_build_ext})
else:
    setup(ext_modules=[
            Extension("range_libc", sources, 
                extra_compile_args = compiler_flags,
                extra_link_args = ["-std=c++11"],
                include_dirs = include_dirs,
                depends=["../includes/*.h"],
                language="c++",)],
        name='range_libc',
        author='Corey Walsh',
        version='0.1',
        cmdclass = {'build_ext': build_ext})
