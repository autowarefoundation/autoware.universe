ament_add_gtest(
    test_predictor
    src/test_predictor.cpp
)
target_include_directories(test_predictor PRIVATE ../include)
target_link_libraries(test_predictor predictor)

ament_add_gtest(
    test_resampler
    src/test_resampler.cpp
)
target_include_directories(test_resampler PRIVATE ../include)
target_link_libraries(test_resampler predictor)