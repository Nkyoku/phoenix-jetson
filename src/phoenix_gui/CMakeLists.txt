cmake_minimum_required(VERSION 3.5)
project(phoenix_gui)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (MSVC)
  add_compile_options(/wd4819)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# add include path
#include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#include_directories(${CMAKE_CURRENT_BINARY_DIR})

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
find_package(OpenGL REQUIRED)

# gui
ament_auto_add_executable(gui
  src/main.cpp
  src/main_window.cpp
  src/node_thread.cpp
  src/image_viewer.cpp
  src/gamepad_thread.cpp
  src/control_pad.cpp
  src/control_pad_scene.cpp
  src/parameter_editor.cpp
  src/diagnostics_viewer.cpp
  src/telemetry_viewer.cpp
  src/system_group.cpp
  src/cintelhex/ihex_copy.c
  src/cintelhex/ihex_parse.c
  src/cintelhex/ihex_record.c
)
target_link_libraries(gui
  Qt5::Widgets
  Qt5::OpenGL
  ${OPENGL_LIBRARIES}
)
target_compile_definitions(gui
  PUBLIC _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
