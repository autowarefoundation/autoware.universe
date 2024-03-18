cmake_minimum_required(VERSION 3.5)

project(rs_driver_demos)

message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)

if (${ENABLE_PCL_POINTCLOUD})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

endif (${ENABLE_PCL_POINTCLOUD})

include_directories(${DRIVER_INCLUDE_DIRS})

add_executable(demo_online
              demo_online.cpp)

target_link_libraries(demo_online
                    ${EXTERNAL_LIBS})

add_executable(demo_online_multi_lidars
              demo_online_multi_lidars.cpp)

target_link_libraries(demo_online_multi_lidars
                    ${EXTERNAL_LIBS})

if(NOT ${DISABLE_PCAP_PARSE})

add_executable(demo_pcap
               demo_pcap.cpp)

target_link_libraries(demo_pcap
                     ${EXTERNAL_LIBS})

endif(NOT ${DISABLE_PCAP_PARSE})

        
