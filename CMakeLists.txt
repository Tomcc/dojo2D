add_subdirectory("dojo")
add_subdirectory("Box2D/liquidfun/Box2D")

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMake")

project("Dojo2D")

option(IWYU "IWYU" OFF)

include(CheckCXXCompilerFlag)
include(dojo/AddDojoIncludes.txt)

include_directories("include/Dojo2D")
include_directories("Box2D/liquidfun/Box2D")

if (WIN32)
    include_directories("include/dojo/win32")
    
    # prevent definition of min and max macros through inclusion of Windows.h
    add_definitions("-DNOMINMAX")
    add_definitions("-DWIN32_LEAN_AND_MEAN")
    add_definitions ("/WX")
    add_definitions ("/WX")
    add_definitions(/FI"${CMAKE_CURRENT_LIST_DIR}/stdafx.h")
else() 
    CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)

    if(COMPILER_SUPPORTS_CXX14)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    else()
        message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
    endif()

    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
    endif()

    add_definitions ("-Wall")
    add_definitions ("-Wno-unknown-pragmas")
    add_definitions ("-Wno-reorder")
    add_definitions ("-Werror")
endif()

file(GLOB common_src    
	"include/Dojo2D/*.h"
    "src/*.cpp"
)

set(dojo_SRC ${common_src} ${platform_src})

add_library(Dojo2D ${dojo_SRC})

target_link_libraries(Dojo2D Dojo Box2D)

if (IWYU)
    find_program(iwyu_path NAMES include-what-you-use iwyu)
    if(NOT iwyu_path)
      message(FATAL_ERROR "Could not find the program include-what-you-use")
    endif()

    set_property(TARGET Dojo2D PROPERTY CXX_INCLUDE_WHAT_YOU_USE "${iwyu_path}")
else()
    include(cotire)

    set(ENV{CCACHE_SLOPPINESS} "pch_defines,time_macros")
    
    set_target_properties(Dojo2D PROPERTIES
        COTIRE_PREFIX_HEADER_IGNORE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_SOURCE_DIR}/include;"
        COTIRE_ADD_UNITY_BUILD FALSE)

    cotire(Dojo2D)
endif()
