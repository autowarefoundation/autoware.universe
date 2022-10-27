# ndt_omp
This package provides an OpenMP-boosted Normal Distributions Transform (and GICP) algorithm derived from pcl. The NDT algorithm is modified to be SSE-friendly and multi-threaded. It can run up to 10 times faster than its original version in pcl.

NOTE: This package's implementation is a customized version of [ndt_omp from koide3](https://github.com/koide3/ndt_omp) for Autoware.

[![Build](https://github.com/koide3/ndt_omp/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/ndt_omp/actions/workflows/build.yml)

### Benchmark (on Core i7-6700K)
```
$ roscd ndt_omp/data
$ rosrun ndt_omp align 251370668.pcd 251371071.pcd
--- pcl::NDT ---
single : 282.222[msec]
10times: 2921.92[msec]
fitness: 0.213937

--- pclomp::NDT (KDTREE, 1 threads) ---
single : 207.697[msec]
10times: 2059.19[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 1 threads) ---
single : 139.433[msec]
10times: 1356.79[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 1 threads) ---
single : 34.6418[msec]
10times: 317.03[msec]
fitness: 0.208511

--- pclomp::NDT (KDTREE, 8 threads) ---
single : 54.9903[msec]
10times: 500.51[msec]
fitness: 0.213937

--- pclomp::NDT (DIRECT7, 8 threads) ---
single : 63.1442[msec]
10times: 343.336[msec]
fitness: 0.214205

--- pclomp::NDT (DIRECT1, 8 threads) ---
single : 17.2353[msec]
10times: 100.025[msec]
fitness: 0.208511
```

Several methods for neighbor voxel search are implemented. If you select pclomp::KDTREE, results will be completely same as the original pcl::NDT. We recommend to use pclomp::DIRECT7 which is faster and stable. If you need extremely fast registration, choose pclomp::DIRECT1, but it might be a bit unstable.

<img src="data/screenshot.png" height="400pix" /><br>
Red: target, Green: source, Blue: aligned

## Related packages
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)

# License

A license from [koide3/ndt_omp](https://github.com/koide3/ndt_omp):
```
BSD 2-Clause License

Copyright (c) 2019, k.koide
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

# Special thanks
- [koide3/ndt_omp](https://github.com/koide3/ndt_omp)