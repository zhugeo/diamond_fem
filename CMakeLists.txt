cmake_minimum_required(VERSION 3.24)
project(DiamondFEM VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(DiamondFEM)

file(GLOB_RECURSE DIAMOND_FEM_SOURCES src/**/*.cpp)
target_sources(DiamondFEM PRIVATE ${DIAMOND_FEM_SOURCES})

include(cmake/CPM.cmake)

CPMAddPackage("gh:fmtlib/fmt#7.1.3")

option(DIAMOND_FEM_BUILD_TESTS "Build tests" OFF)
if(DIAMOND_FEM_BUILD_TESTS)
    CPMAddPackage(
        NAME GTest
        GITHUB_REPOSITORY google/googletest
        VERSION 1.17.0
    )
    find_package(GTest REQUIRED)
    include(GoogleTest)
    enable_testing()
    cmake_minimum_required(VERSION 3.24)

    file(GLOB_RECURSE TEST_SOURCES tests/**/*.cpp)

    add_executable(DiamondFEMTests ${TEST_SOURCES})

    target_link_libraries(DiamondFEMTests GTest::gtest_main DiamondFEM)
    target_include_directories(DiamondFEMTests PRIVATE ${gtest_SOURCE_DIR}/include)

    gtest_discover_tests(DiamondFEMTests)
endif()

find_package(CGAL REQUIRED COMPONENTS Qt6)

target_include_directories(DiamondFEM PUBLIC include)
target_link_libraries(DiamondFEM CGAL::CGAL CGAL::CGAL_Basic_viewer fmt::fmt)
