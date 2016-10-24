add_subdirectory("dojo")
add_subdirectory("Box2D/liquidfun/Box2D")

cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/dojo")

project("Dojo2D")

option(IWYU "IWYU" OFF)

include(dojo/AddDojoIncludes.cmake)
INCLUDE(dojo/MSVCSetup.cmake)
include(dojo/ClangGCCSetup.cmake)

include_directories("include/Dojo2D")
include_directories("Box2D/liquidfun/Box2D")

set(COMPILE_DEFINITIONS_RELEASE "${COMPILE_DEFINITIONS_RELEASE} -DPUBLISH")

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
    
    set_target_properties(Dojo2D PROPERTIES
        COTIRE_PREFIX_HEADER_IGNORE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src;${CMAKE_CURRENT_SOURCE_DIR}/include;"
        COTIRE_ADD_UNITY_BUILD FALSE)

    cotire(Dojo2D)
endif()
