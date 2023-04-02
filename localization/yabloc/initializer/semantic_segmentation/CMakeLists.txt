cmake_minimum_required(VERSION 3.5)
project(semantic_segmentation)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ===================================================
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# ===================================================
file(MAKE_DIRECTORY data)
ament_auto_package(INSTALL_TO_SHARE data launch)

install(PROGRAMS
    src/semseg_core.py
    src/semseg_node.py
    src/semseg_server.py
    DESTINATION lib/${PROJECT_NAME}
)
