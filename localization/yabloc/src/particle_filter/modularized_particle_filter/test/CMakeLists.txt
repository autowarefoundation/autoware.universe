# Test for prediction
ament_add_gtest(
    prediction_test
    src/test_prediction.cpp
)
target_include_directories(prediction_test PRIVATE ../include)
target_link_libraries(prediction_test predictor)