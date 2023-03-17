add_rules("mode.debug", "mode.release")
set_languages("c++20")
add_requires("sfml","imgui","imgui-sfml")

target("Physics2D")
    set_kind("static")
    add_files("Physics2D-TestBed-SFML/dependencies/Physics2D/*.cpp")


target("Physics2D-TestBed-SFML")
    set_kind("binary")
    add_deps("Physics2D")
    add_headerfiles("Physics2D-TestBed-SFML/dependencies/Physics2D/*.h")
    add_headerfiles("Physics2D-TestBed-SFML/include/*.h")
    add_includedirs("Physics2D-TestBed-SFML/include")
    add_files("Physics2D-TestBed-SFML/source/*.cpp")
    add_files("Physics2D-TestBed-SFML/main.cpp")
    add_packages("sfml","imgui","imgui-sfml")