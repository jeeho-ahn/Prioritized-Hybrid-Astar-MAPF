cmake_minimum_required(VERSION 3.16)

project(PHASTAR LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

configure_file (${CMAKE_CURRENT_SOURCE_DIR}/include/config.h.in ${CMAKE_CURRENT_SOURCE_DIR}/include/config.h @ONLY)

## Path Visualization
# First, try to find Qt6 with the Widgets component quietly.
find_package(Qt6 COMPONENTS Widgets QUIET)

if (Qt6_FOUND)
    message(STATUS "Found Qt6, using Qt6::Widgets")
    set(QT_WIDGET_TARGET Qt6::Widgets)
    # Optionally, define a preprocessor symbol to help your source code distinguish versions.
    add_compile_definitions(USE_QT6)
    if(APPLE)
        get_target_property(_wrap_opengl_libs WrapOpenGL::WrapOpenGL INTERFACE_LINK_LIBRARIES)
        message(STATUS "WrapOpenGL INTERFACE_LINK_LIBRARIES before filter: ${_wrap_opengl_libs}")
        list(FILTER _wrap_opengl_libs EXCLUDE REGEX ".*AGL.*")
        message(STATUS "WrapOpenGL INTERFACE_LINK_LIBRARIES after filter: ${_wrap_opengl_libs}")
        set_target_properties(WrapOpenGL::WrapOpenGL PROPERTIES INTERFACE_LINK_LIBRARIES "${_wrap_opengl_libs}")
    endif()
else()
    message(STATUS "Qt6 not found; trying Qt5")
    find_package(Qt5 COMPONENTS Widgets REQUIRED)
    message(STATUS "Found Qt5, using Qt5::Widgets")
    set(QT_WIDGET_TARGET Qt5::Widgets)
    add_compile_definitions(USE_QT5)
endif()

include_directories(include include/fromReloPush)

add_executable(phastar_demo
    src/PHAstar_demo.cpp
)

add_executable(phastar_push_demo
    src/PHAstar_push_demo.cpp
    src/fromReloPush/base64.cpp # for importing task sequence from file
)

target_link_libraries(phastar_demo PRIVATE
    ${QT_WIDGET_TARGET}
)

target_link_libraries(phastar_push_demo PRIVATE
    ${QT_WIDGET_TARGET}
)
