cmake_minimum_required(VERSION 3.7)
project(myapp)

set(CMAKE_CXX_STANDARD 17)
set(VULKAN_SDK_PATH "~/VulkanSDK/1.3.296.0/macOS")

find_package(vsg REQUIRED)
find_package(Vulkan REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(glslang REQUIRED HINTS "${VULKAN_SDK_PATH}/lib/cmake")


include_directories(/opt/homebrew/Cellar/ode/0.16.5_1/include)
link_directories(/opt/homebrew/Cellar/ode/0.16.5_1/lib)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${VSG_INCLUDE_DIR}
    ${ODE_INCLUDE_DIR}
    ${Vulkan_INCLUDE_DIRS}
    ${GLFW_INCLUDE_DIRS}
    ${GLSLANG_INCLUDE_DIRS}  # Add this line
)

# set(SRC_FILES
#     main.cpp
#     base.cpp
#     colorschema.cpp
#     primitive.cpp
#     substance.cpp
#     vsghandle.cpp
#     vsgprimitive.cpp
#     odehandle.cpp
#     backcaller.cpp
# )
# Automatically find all .cpp files in the source directory
file(GLOB SRC_FILES "${CMAKE_SOURCE_DIR}/*.cpp")

add_executable(myapp ${SRC_FILES})

if (vsgXchange_FOUND)
    target_compile_definitions(myapp PRIVATE vsgXchange_FOUND)
    target_link_libraries(myapp vsgXchange::vsgXchange)
endif()

target_link_libraries(myapp
    vsg::vsg
    ode
    ${Vulkan_LIBRARIES}
    glfw
    glslang  # Add this line
    SPIRV    # Add this line
    "-framework OpenGL"
    "-framework Cocoa"
    "-framework IOKit"
    "-framework CoreVideo"
)
