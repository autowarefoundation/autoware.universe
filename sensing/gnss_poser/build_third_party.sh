## Build third party -- GeographicLib

cd third_party
cd GeographicLib
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
cd ../../
