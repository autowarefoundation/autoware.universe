
cmake_minimum_required(VERSION 3.5)

project(rs_driver_test)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${DRIVER_INCLUDE_DIRS})

add_definitions("-DUNIT_TEST")

add_executable(rs_driver_test
              rs_driver_test.cpp
              buffer_test.cpp
              sync_queue_test.cpp
              trigon_test.cpp
              basic_attr_test.cpp
              section_test.cpp
              chan_angles_test.cpp
              split_strategy_test.cpp
              single_return_block_iterator_test.cpp
              dual_return_block_iterator_test.cpp
              ab_dual_return_block_iterator_test.cpp
              rs16_single_return_block_iterator_test.cpp
              rs16_dual_return_block_iterator_test.cpp
              decoder_test.cpp
              decoder_rsbp_test.cpp
              decoder_rs32_test.cpp
              decoder_rs16_test.cpp)

target_link_libraries(rs_driver_test
                      ${GTEST_LIBRARIES}
                      ${EXTERNAL_LIBS})
        
