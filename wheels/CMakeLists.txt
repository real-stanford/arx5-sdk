cmake_minimum_required( VERSION 3.16 )

project( arx5_interface )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # global PIC

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS container)


execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('purelib'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

find_package(pybind11 REQUIRED PATHS ${PYTHON_SITE_PACKAGES}/pybind11/share/cmake/pybind11)

find_package(Python3 COMPONENTS Interpreter Development)


add_compile_definitions(SDK_ROOT="${CMAKE_BINARY_DIR}/..")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/lib/aarch64)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/lib/x86_64)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(
    $ENV{CONDA_PREFIX}/include/
    $ENV{CONDA_PREFIX}/include/urdfdom_headers
)

pybind11_add_module(arx5_interface
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/python/arx5_pybind.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/app/joint_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/app/cartesian_controller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/app/controller_base.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/src/utils.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/include/libcan)
add_compile_options(-pthread)

target_link_libraries(arx5_interface PRIVATE
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    spdlog::spdlog
    Eigen3::Eigen
    Threads::Threads
    kdl_parser
    orocos-kdl
    pthread
    soem
)


target_include_directories(arx5_interface PUBLIC ${EIGEN3_INCLUDE_DIRS})

set_target_properties(arx5_interface PROPERTIES 
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/arx5_interface/python"
)

# Add this to ensure the wheel is built with the correct ABI flags
if(DEFINED ENV{CONDA_PREFIX})
    set(PYTHON_ABI ".cp${Python_VERSION_MAJOR}${Python_VERSION_MINOR}-cp${Python_VERSION_MAJOR}${Python_VERSION_MINOR}")
else()
    execute_process(
        COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('SOABI'))"
        OUTPUT_VARIABLE PYTHON_ABI
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
endif()