cmake_minimum_required(VERSION 3.16)
project(Physics2D)
set(CMAKE_CXX_STANDARD 20)

# Find dependencies
find_package(imgui CONFIG REQUIRED)
find_package(SFML CONFIG REQUIRED COMPONENTS graphics system window)
find_package(ImGui-SFML CONFIG REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

# Set target Physics2D
add_library(Physics2D)
target_compile_definitions(Physics2D PUBLIC "$<$<CONFIG:DEBUG>:DEBUG>")
target_compile_definitions(Physics2D PUBLIC "$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:NDEBUG>")
target_compile_options(Physics2D PUBLIC "$<$<CONFIG:DEBUG>:-O0>")
target_compile_options(Physics2D PUBLIC "$<$<OR:$<CONFIG:RELEASE>,$<CONFIG:PROFILE>>:-O3>")

target_include_directories(Physics2D PUBLIC "Physics2D-TestBed-SFML/dependencies/Physics2D/include")

file(GLOB_RECURSE PHYSICS2D_COLLISION_SOURCES "Physics2D-TestBed-SFML/dependencies/Physics2D/source/collision/*.cpp")
file(GLOB_RECURSE PHYSICS2D_DYNAMICS_SOURCES "Physics2D-TestBed-SFML/dependencies/Physics2D/source/dynamics/*.cpp")
file(GLOB_RECURSE PHYSICS2D_MATH_SOURCES "Physics2D-TestBed-SFML/dependencies/Physics2D/source/math/*.cpp")
file(GLOB_RECURSE PHYSICS2D_OTHER_SOURCES "Physics2D-TestBed-SFML/dependencies/Physics2D/source/other/*.cpp")

target_sources(Physics2D PRIVATE 
    ${PHYSICS2D_COLLISION_SOURCES}
    ${PHYSICS2D_DYNAMICS_SOURCES}
    ${PHYSICS2D_MATH_SOURCES}
    ${PHYSICS2D_OTHER_SOURCES}
)

# Set target Physics2D-TestBed-SFML
add_executable(Physics2D-TestBed-SFML)
target_include_directories(Physics2D-TestBed-SFML PRIVATE 
    "Physics2D-TestBed-SFML/include" 
    "Physics2D-TestBed-SFML/dependencies/Physics2D/include"
)

file(GLOB_RECURSE TESTBED_SOURCES "Physics2D-TestBed-SFML/source/*.cpp")

target_sources(Physics2D-TestBed-SFML PRIVATE 
    ${TESTBED_SOURCES}
    "Physics2D-TestBed-SFML/main.cpp"
)
target_link_libraries(Physics2D-TestBed-SFML PRIVATE 
    Physics2D
    imgui::imgui
    sfml-system sfml-network sfml-graphics sfml-window
    ImGui-SFML::ImGui-SFML
    Eigen3::Eigen
    Ceres::ceres
)

add_custom_command(TARGET Physics2D-TestBed-SFML POST_BUILD 
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/Physics2D-TestBed-SFML/font
        $<TARGET_FILE_DIR:Physics2D-TestBed-SFML>/font
)
