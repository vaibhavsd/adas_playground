cmake_minimum_required(VERSION 3.20)
project(adas_playground LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(ADAS_TSAN "Build with ThreadSanitizer (find data races)" OFF)

find_package(Threads REQUIRED)

# Header-only library target: everything lives under include/adas.
add_library(adas INTERFACE)
target_include_directories(adas INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(adas INTERFACE Threads::Threads)
if(MSVC)
  target_compile_options(adas INTERFACE /W4 /permissive-)
else()
  target_compile_options(adas INTERFACE -Wall -Wextra -Wpedantic)
  if(ADAS_TSAN)
    target_compile_options(adas INTERFACE -fsanitize=thread -g -O1)
    target_link_options(adas INTERFACE -fsanitize=thread)
  endif()
endif()

add_executable(adas_sim src/main.cpp)
target_link_libraries(adas_sim PRIVATE adas)

add_executable(adas_tests tests/test_main.cpp)
target_link_libraries(adas_tests PRIVATE adas)

enable_testing()
add_test(NAME unit COMMAND adas_tests)
