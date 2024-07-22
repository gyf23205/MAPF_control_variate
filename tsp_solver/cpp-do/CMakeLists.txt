cmake_minimum_required(VERSION 2.8.3)
project(cppado)

# find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

file(GLOB_RECURSE ALL_HDRS "include/*.hpp")
file(GLOB_RECURSE ALL_SRCS "source/*.cpp")

include_directories(DIRECTORY
  include/${PROJECT_NAME}
  include
)

add_library(${PROJECT_NAME} SHARED
  ${ALL_SRCS}
        include/msp.hpp source/msp.cpp test/test_Algorithm.cpp include/Algo.hpp source/Algo.cpp include/Algo_util.hpp source/Algo_util.cpp)

set(test_cpp_dir "test/")
set(test_cpp_files "")
file(GLOB_RECURSE test_cpp_files "${test_cpp_dir}/*.cpp")
foreach(test_cpp_file ${test_cpp_files})
  get_filename_component(test_cpp_name ${test_cpp_file} NAME_WE)
  #ADD_EXECUTABLE(${test_cpp_name} ${ALL_SRCS} ${test_cpp_dir}/${test_cpp_name}.cpp)
  ADD_EXECUTABLE(${test_cpp_name} ${test_cpp_dir}/${test_cpp_name}.cpp)
  TARGET_LINK_LIBRARIES(${test_cpp_name}
    ${PROJECT_NAME}
  )
endforeach(test_cpp_file ${test_cpp_files})


set(prog_cpp_dir "program/")
set(prog_cpp_files "")
file(GLOB_RECURSE prog_cpp_files "${prog_cpp_dir}/*.cpp")
foreach(prog_cpp_file ${prog_cpp_files})
  get_filename_component(prog_cpp_name ${prog_cpp_file} NAME_WE)
  #ADD_EXECUTABLE(${test_cpp_name} ${ALL_SRCS} ${test_cpp_dir}/${test_cpp_name}.cpp)
  ADD_EXECUTABLE(${prog_cpp_name} ${prog_cpp_dir}/${prog_cpp_name}.cpp program/api.cpp include/api_util.hpp)
  TARGET_LINK_LIBRARIES(${prog_cpp_name}
    ${PROJECT_NAME}
  )
endforeach(prog_cpp_file ${prog_cpp_files})
