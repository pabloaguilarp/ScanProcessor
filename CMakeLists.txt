cmake_minimum_required(VERSION 3.22)
project(ScanProcessor)

set(CMAKE_CXX_STANDARD 17)

# Find PCL
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization)

# Find JSON
include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
FetchContent_MakeAvailable(json)

# Find boost
find_package(Boost COMPONENTS program_options)

# Create target
add_executable(ScanProcessor main.cpp ScanIO/ScanIO.cpp ScanIO/ScanIO.h User/User.cpp User/User.h common.h argparse.h)

# Link PCL
target_include_directories(ScanProcessor PUBLIC ${PCL_INCLUDE_DIRS})
target_link_directories(ScanProcessor PUBLIC ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
target_link_libraries(ScanProcessor PUBLIC ${PCL_LIBRARIES})

# Link JSON
target_link_libraries(ScanProcessor PUBLIC nlohmann_json::nlohmann_json)

# Link Boost
if(Boost_FOUND)
    target_include_directories(ScanProcessor PUBLIC ${Boost_INCLUDE_DIRS})
    target_link_libraries(ScanProcessor PUBLIC ${Boost_LIBRARIES})
endif()
