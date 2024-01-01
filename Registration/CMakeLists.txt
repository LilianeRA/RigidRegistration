cmake_minimum_required(VERSION 3.5)

project(Registration)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 20) # C++20...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

#add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

#if(UNIX)
#	set(CMAKE_C_COMPILER /usr/bin/gcc-10 CACHE PATH "" FORCE)
#	set(CMAKE_C_COMPILER /usr/bin/g++-10 CACHE PATH "" FORCE)
#	set(CMAKE_CXX_COMPILER /usr/bin/g++-10 CACHE PATH "" FORCE)
#else() # WIN32
#	# Add module path to find Eigen. The FindXXX.cmake
#	list(APPEND CMAKE_MODULE_PATH D:/ProgrammingProjects )
#	list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}") # Eigen3Config.cmake
#endif()

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3") # its being ignored in Visual Studio...
set(CMAKE_POSITION_INDEPENDENT_CODE OFF)
option(OPTIMIZE_FOR_NATIVE "Build with -march=native" ON)

#--- CMake configuration
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

#--- OpenMP (cannot work in clang)
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()


#--- Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#--- GLAD 
find_package(GLAD REQUIRED)
include_directories(${GLAD_INCLUDE_DIR})
#--- GLM
find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIR})
#--- Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
#--- GL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIR})
#--- GLFW
find_package(GLFW REQUIRED)
include_directories(${GLFW_INCLUDE_DIR})
#--- ImGUI
find_package(IMGUI REQUIRED)
include_directories(${IMGUI_INCLUDE_DIR})

#--- My Visualizer3D. Will be optional
find_package(Visualizer3D REQUIRED)
include_directories(${Visualizer3D_INCLUDE_DIR})

set(LIBRARIES ${IMGUI_LIBRARY} ${GLFW_LIBRARY} ${GLAD_LIBRARY} ${Visualizer3D_LIBRARY})

#--- Build example
include_directories(.)
include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)
file(GLOB HEADERS "*.h" "${CMAKE_SOURCE_DIR}/include/*.h")
file(GLOB SOURCES "*.cpp" "${CMAKE_SOURCE_DIR}/src/*.cpp")
add_executable(Registration ${SOURCES} ${HEADERS})
target_link_libraries(Registration PUBLIC Eigen3::Eigen ${LIBRARIES} )




# git status
# git add *
# git add .gitignore
# git config --global user.name "gitusername"
# git config --global user.email "gitemail"
# git commit -a -m "First commit"
# git push
