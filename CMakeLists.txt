cmake_minimum_required(VERSION 3.18)

project(3DPrintPlanner LANGUAGES CXX)

find_package(Qt6 REQUIRED COMPONENTS Gui Core Xml)
find_package(Boost REQUIRED
             COMPONENTS system filesystem thread)
find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


include_directories(.)
include_directories(./QMeshLib)
include_directories(./QMeshLib/OpenMesh)
include_directories(./GLKLib)
include_directories(./GLUILib/include)
include_directories(./Utils)


################ These are the paths of TetWild libraries ##############
include_directories(../TetWild/include)
include_directories(../TetWild/extern/libigl/include)

link_directories(../TetWild/build)
link_directories(../TetWild/build/lib)
link_directories(../TetWild/build/extern/fmt)
link_directories(../TetWild/build/extern/pymesh)
#########################################################################


link_directories(/usr/local/lib)
link_directories(/usr/lib/x86_64-linux-gnu)

file(GLOB MAIN_SOURCE_FILES "${CMAKE_SOURCE_DIR}/*.cpp")
file(GLOB MAIN_HEADER_FILES "${CMAKE_SOURCE_DIR}/*.h")
file(GLOB_RECURSE QMESHLIB_SOURCE "${CMAKE_SOURCE_DIR}/QMeshLib/*.cpp" "${CMAKE_SOURCE_DIR}/QMeshLib/*.cc")
file(GLOB_RECURSE QMESHLIB_HEADER "${CMAKE_SOURCE_DIR}/QMeshLib/*.h" "${CMAKE_SOURCE_DIR}/QMeshLib/*.hh")
file(GLOB_RECURSE UTILS_SOURCE "${CMAKE_SOURCE_DIR}/Utils/*/*.cpp")
file(GLOB_RECURSE UTILS_HEADER "${CMAKE_SOURCE_DIR}/Utils/*/*.h")
file(GLOB_RECURSE GLK_SOURCE "${CMAKE_SOURCE_DIR}/GLKLib/*.cpp")
file(GLOB_RECURSE GLK_HEADER "${CMAKE_SOURCE_DIR}/GLKLib/*.h")
file(GLOB_RECURSE PQP_SOURCE "${CMAKE_SOURCE_DIR}/PQPLib/*.cpp")
file(GLOB_RECURSE PQP_HEADER "${CMAKE_SOURCE_DIR}/PQPLib/*.h")
file(GLOB_RECURSE GLUI_SOURCE_1 "${CMAKE_SOURCE_DIR}/GLUILib/*.c")
file(GLOB_RECURSE GLUI_SOURCE_2 "${CMAKE_SOURCE_DIR}/GLUILib/*.cpp")
file(GLOB_RECURSE GLUI_HEADER "${CMAKE_SOURCE_DIR}/GLUILib/*.h")

add_executable(${PROJECT_NAME} ${MAIN_SOURCE_FILES} ${MAIN_HEADER_FILES} ${QMESHLIB_SOURCE} ${QMESHLIB_HEADER}
 ${UTILS_SOURCE} ${UTILS_HEADER} ${GLK_SOURCE} ${GLK_HEADER} ${PQP_SOURCE} ${PQP_HEADER}
 ${GLUI_SOURCE_1} ${GLUI_SOURCE_2} ${GLUI_HEADER})

target_link_libraries(${PROJECT_NAME} GLU glut GLEW)
target_link_libraries(${PROJECT_NAME} Qt6::Core Qt6::Gui Qt6::Xml)
target_link_libraries(${PROJECT_NAME} OpenGL::GL)
target_link_libraries(${PROJECT_NAME} tetwild fmt pymesh_tiny gmp mpfr geogram boost_thread)

# openmp and -o3 parrellel
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -fopenmp")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
