
cmake_minimum_required (VERSION 3.8)
set(PHYSICS2D_ALL)
file(GLOB_RECURSE PHYSICS2D_ALL ./dependencies/Physics2D/*.*)

set(TESTBED_HEADER_ALL)
file(GLOB_RECURSE TESTBED_HEADER_ALL ./include/*.h)
set(TESTBED_SOURCE_ALL)
file(GLOB_RECURSE TESTBED_SOURCE_ALL ./source/*.cpp)

add_executable (Physics2D-TestBed-SFML "main.cpp" ${PHYSICS2D_ALL} ${TESTBED_HEADER_ALL} 
${TESTBED_SOURCE_ALL})


find_package(SFML COMPONENTS system window graphics CONFIG REQUIRED)
target_link_libraries(Physics2D-TestBed-SFML PRIVATE sfml-system sfml-network sfml-graphics sfml-window)

find_package(imgui CONFIG REQUIRED)
target_link_libraries(Physics2D-TestBed-SFML PRIVATE imgui::imgui)

find_package(ImGui-SFML CONFIG REQUIRED)
target_link_libraries(Physics2D-TestBed-SFML PRIVATE ImGui-SFML::ImGui-SFML)

find_package(benchmark CONFIG REQUIRED)
target_link_libraries(Physics2D-TestBed-SFML PRIVATE benchmark::benchmark benchmark::benchmark_main)