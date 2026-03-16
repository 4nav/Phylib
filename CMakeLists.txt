cmake_minimum_required(VERSION 3.16)
project(Phylib VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

# ─── GLFW (windowing + input) ─────────────────────────────────────────────────
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL        OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# ─── GLM (header-only math) ───────────────────────────────────────────────────
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        0.9.9.8
    GIT_SHALLOW    TRUE
    GIT_PROGRESS   TRUE
)
FetchContent_MakeAvailable(glm)

# ─── OpenGL ───────────────────────────────────────────────────────────────────
find_package(OpenGL REQUIRED)

# ─── GLEW (OpenGL function loader) ────────────────────────────────────────────
# Install via: sudo apt install libglew-dev  OR  brew install glew
find_package(GLEW REQUIRED)

# ─── Source files ─────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/Shader.cpp
    src/RigidBody.cpp
    src/Collision.cpp
    src/Pendulum.cpp
    src/Spring.cpp
    src/PhysicsWorld.cpp
    src/Renderer.cpp
)

add_executable(Phylib ${SOURCES})

target_include_directories(Phylib PRIVATE src)

target_link_libraries(Phylib PRIVATE
    OpenGL::GL
    glfw
    glm
    GLEW::GLEW
)

# ─── Compiler warnings ────────────────────────────────────────────────────────
if(MSVC)
    target_compile_options(Phylib PRIVATE /W4 /WX)
else()
    target_compile_options(Phylib PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ─── Copy shaders to build directory after build ──────────────────────────────
add_custom_command(
    TARGET Phylib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/shaders
        $<TARGET_FILE_DIR:Phylib>/shaders
    COMMENT "Copying shaders to build directory"
)
