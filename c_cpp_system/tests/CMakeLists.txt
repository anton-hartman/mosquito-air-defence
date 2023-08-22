# List your test files here
set(TEST_SOURCES
    gpio/gpio_test.cpp
    detection/detection_test.cpp
    motors/controller_test.cpp
)

add_executable(MosquitoAirDefenceTests ${TEST_SOURCES})
target_link_libraries(MosquitoAirDefenceTests gtest gtest_main)
