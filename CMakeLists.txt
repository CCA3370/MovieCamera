cmake_minimum_required(VERSION 3.16)
project(MovieCamera VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform detection
if(WIN32)
    set(PLATFORM_NAME "win")
    add_definitions(-DIBM=1 -DLIN=0 -DAPL=0)
elseif(APPLE)
    set(PLATFORM_NAME "mac")
    add_definitions(-DIBM=0 -DLIN=0 -DAPL=1)
else()
    set(PLATFORM_NAME "lin")
    add_definitions(-DIBM=0 -DLIN=1 -DAPL=0)
endif()

# SDK version definitions
add_definitions(-DXPLM200=1 -DXPLM210=1 -DXPLM300=1 -DXPLM301=1 -DXPLM302=1 -DXPLM400=1 -DXPLM410=1 -DXPLM420=1)

# Paths
set(SDK_DIR "${CMAKE_SOURCE_DIR}/SDK")
set(IMGUI_DIR "${CMAKE_SOURCE_DIR}/imgui")

# Source files
set(PLUGIN_SOURCES
    src/MovieCamera.cpp
    src/ImgWindow/ImgWindow.cpp
    src/ImgWindow/ImgFontAtlas.cpp
)

# ImGui sources
set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
)

# Create shared library
add_library(MovieCamera SHARED ${PLUGIN_SOURCES} ${IMGUI_SOURCES})

# Include directories
target_include_directories(MovieCamera PRIVATE
    ${SDK_DIR}/CHeaders/XPLM
    ${SDK_DIR}/CHeaders/Widgets
    ${SDK_DIR}/CHeaders/Wrappers
    ${IMGUI_DIR}
    ${CMAKE_SOURCE_DIR}/src/ImgWindow
)

# Platform-specific settings
if(WIN32)
    target_link_directories(MovieCamera PRIVATE ${SDK_DIR}/Libraries/Win)
    target_link_libraries(MovieCamera PRIVATE XPLM_64 XPWidgets_64 opengl32)
    set_target_properties(MovieCamera PROPERTIES
        OUTPUT_NAME "MovieCamera"
        SUFFIX ".xpl"
    )
elseif(APPLE)
    find_library(XPLM_LIBRARY XPLM PATHS ${SDK_DIR}/Libraries/Mac REQUIRED)
    find_library(XPWIDGETS_LIBRARY XPWidgets PATHS ${SDK_DIR}/Libraries/Mac REQUIRED)
    find_library(OPENGL_LIBRARY OpenGL REQUIRED)
    target_link_libraries(MovieCamera PRIVATE ${XPLM_LIBRARY} ${XPWIDGETS_LIBRARY} ${OPENGL_LIBRARY})
    set_target_properties(MovieCamera PROPERTIES
        OUTPUT_NAME "MovieCamera"
        SUFFIX ".xpl"
        BUNDLE TRUE
    )
else()
    # Linux
    # For X-Plane plugins on Linux, we link against the .so files that come with the SDK
    # The plugin is loaded by X-Plane which provides the actual implementations
    set(XPLM_LIB "${SDK_DIR}/Libraries/Lin/XPLM_64.so")
    set(XPWIDGETS_LIB "${SDK_DIR}/Libraries/Lin/XPWidgets_64.so")
    
    target_link_libraries(MovieCamera PRIVATE ${XPLM_LIB} ${XPWIDGETS_LIB})
    set_target_properties(MovieCamera PROPERTIES
        OUTPUT_NAME "MovieCamera"
        SUFFIX ".xpl"
        PREFIX ""
    )
    # Position independent code for shared library
    set_target_properties(MovieCamera PROPERTIES POSITION_INDEPENDENT_CODE ON)
    
    # Set rpath for development
    set_target_properties(MovieCamera PROPERTIES
        BUILD_RPATH "${SDK_DIR}/Libraries/Lin"
        INSTALL_RPATH ""
    )
endif()

# Compiler flags
if(MSVC)
    target_compile_options(MovieCamera PRIVATE /W3)
else()
    target_compile_options(MovieCamera PRIVATE -Wall -Wextra -Wno-unused-parameter)
    if(NOT APPLE)
        target_link_options(MovieCamera PRIVATE -static-libgcc -static-libstdc++)
    endif()
endif()

# Hide symbols by default
if(NOT WIN32)
    set_target_properties(MovieCamera PROPERTIES CXX_VISIBILITY_PRESET hidden)
endif()
