cmake_minimum_required (VERSION 3.16)
project (homework1xjb)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)


find_package(GTest REQUIRED)


include_directories(include)
add_library(fkin_1x fkin_1x.cpp)
target_link_libraries (fkin_1x Eigen3::Eigen)
add_executable(fkin_main main_1x.cpp)
target_link_libraries(fkin_main fkin_1x)
enable_testing()

add_executable(
  kin_tests
  tests.cpp
)
target_link_libraries(
    kin_tests
    fkin_1x
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(kin_tests)