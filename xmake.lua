add_rules("mode.debug", "mode.release", "mode.profile")
set_languages("c++20")
add_requires("sfml","imgui","imgui-sfml")

target("Physics2D")
    if is_mode("debug") then 
        set_kind("static")
        add_defines("DEBUG")
        set_symbols("debug")
        set_optimize("none")
    end 
    if is_mode("release", "profile") then 
        set_kind("shared")
        add_defines("NDEBUG")
        set_symbols("hidden")
        set_optimize("fastest")
    end

    add_headerfiles("Physics2D-TestBed-SFML/dependencies/Physics2D/include/*.h")
    add_includedirs("Physics2D-TestBed-SFML/dependencies/Physics2D/include")
    add_files("Physics2D-TestBed-SFML/dependencies/Physics2D/source/*.cpp")


target("Physics2D-TestBed-SFML")
    set_kind("binary")
    add_headerfiles("Physics2D-TestBed-SFML/dependencies/Physics2D/include/*.h")
    add_headerfiles("Physics2D-TestBed-SFML/include/**.h")
    add_includedirs("Physics2D-TestBed-SFML/include")
    add_includedirs("Physics2D-TestBed-SFML/dependencies/Physics2D/include")
    add_files("Physics2D-TestBed-SFML/source/*.cpp")
    add_files("Physics2D-TestBed-SFML/main.cpp")
    add_packages("sfml","imgui","imgui-sfml")
    add_deps("Physics2D")