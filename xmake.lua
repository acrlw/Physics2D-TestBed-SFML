add_rules("mode.debug", "mode.release")
set_languages("c++20")
add_requires("sfml","imgui","imgui-sfml")

target("Physics2D")
    set_kind("shared")
    add_files("Physics2D-TestBed-SFML/dependencies/Physics2D/*.cpp")


target("Physics2D-TestBed-SFML")
    set_kind("binary")
    add_includedirs("Physics2D-TestBed-SFML/include")
    add_files("Physics2D-TestBed-SFML/source/*.cpp")
    add_files("Physics2D-TestBed-SFML/main.cpp")
    add_packages("sfml","imgui","imgui-sfml")
    add_deps("Physics2D")