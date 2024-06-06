# CMake minimum version
cmake_minimum_required(VERSION 3.16)

# project name, version, description, language
project(
  Astar  #Edit
  VERSION 0.1
  DESCRIPTION "Test for Astar"
  LANGUAGES CXX)

#include_directories(include)

#Find Boost  (apt-get install libboost-all-dev)
ADD_DEFINITIONS(-DBOOST_LOG_DYN_LINK)
find_package( Boost 1.49.0 REQUIRED COMPONENTS program_options system filesystem log log_setup)
include_directories(${Boost_INCLUDE_DIRS})


include_directories("${CMAKE_SOURCE_DIR}/inc") #Edit
#file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/cpp/*.cpp") #Edit
file(GLOB SOURCES "${CMAKE_SOURCE_DIR}/cpp/Agents.cpp"
              "cpp/MapSearchNode.cpp") #Edit

add_executable(lifelong ${SOURCES} ) #lifelong edit (executable file)
target_link_libraries(lifelong ${Boost_LIBRARIES})
# target_compile_definitions(lifelong PRIVATE PYTHON=${PYTHON_FLAG})

#Copy data files to build directory
# file(COPY ${CMAKE_SOURCE_DIR}/ConflictBasedSearch/data DESTINATION ${CMAKE_BINARY_DIR})

add_custom_target(clean_all
    COMMAND ${CMAKE_BUILD_TOOL} clean
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/lifelong
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/build
    COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_BINARY_DIR}/*.so
)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES 
    "${CMAKE_BINARY_DIR}/CMakeCache.txt;
     ${CMAKE_BINARY_DIR}/lifelong;
     ${CMAKE_BINARY_DIR}/build;
     ${CMAKE_BINARY_DIR}/*.so")
