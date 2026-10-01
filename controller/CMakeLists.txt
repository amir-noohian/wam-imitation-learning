cmake_minimum_required(VERSION 3.10)
project(wam_teach LANGUAGES CXX)
find_package(Barrett REQUIRED)
link_directories(${BARRETT_LIBRARY_DIRS})
add_executable(wam_teach wam_teach.cpp)
# Installed libbarrett/libconfig headers use exception specifications removed in C++17.
# A minimum compile feature alone permits the compiler to keep a newer default.
set_target_properties(wam_teach PROPERTIES
    CXX_STANDARD 11
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)
target_include_directories(wam_teach PRIVATE ${BARRETT_INCLUDE_DIRS})
target_link_libraries(wam_teach ${BARRETT_LIBRARIES})
add_definitions(${BARRETT_DEFINITIONS})

add_executable(wam_replay wam_replay.cpp)
set_target_properties(wam_replay PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_include_directories(wam_replay PRIVATE ${BARRETT_INCLUDE_DIRS})
target_link_libraries(wam_replay ${BARRETT_LIBRARIES})

add_executable(wam_policy wam_policy.cpp)
set_target_properties(wam_policy PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_include_directories(wam_policy PRIVATE ${BARRETT_INCLUDE_DIRS})
target_link_libraries(wam_policy ${BARRETT_LIBRARIES})

include(CTest)
if(BUILD_TESTING)
    add_executable(test_replay test_replay.cpp)
    set_target_properties(test_replay PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
    target_include_directories(test_replay PRIVATE ${BARRETT_INCLUDE_DIRS})
    target_link_libraries(test_replay ${BARRETT_LIBRARIES})
    add_test(NAME replay_validation COMMAND test_replay)
endif()
