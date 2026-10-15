cmake_minimum_required(VERSION 3.16)
project(b2_terrain_planner LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(B2_TERRAIN_STANDALONE "Build the CPU planner without ROS or GPU dependencies" OFF)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
set(BLA_SIZEOF_INTEGER 4)
find_package(LAPACK REQUIRED)
find_package(RBDL REQUIRED COMPONENTS URDFREADER)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
if(NOT RBDL_URDFREADER_FOUND)
    message(FATAL_ERROR "RBDL URDF reader is required")
endif()
find_path(OSQP_INCLUDE_DIR osqp.h PATH_SUFFIXES osqp REQUIRED)
find_library(OSQP_LIBRARY osqp REQUIRED)
include(CheckCSourceCompiles)
set(CMAKE_REQUIRED_INCLUDES "${OSQP_INCLUDE_DIR}")
set(CMAKE_REQUIRED_LIBRARIES "${OSQP_LIBRARY}")
check_c_source_compiles("#include <osqp.h>
int main(void) { OSQPWorkspace *w = 0; OSQPData d; OSQPSettings s;
  _Static_assert(sizeof(c_float) == sizeof(double), \"OSQP double precision required\");
  osqp_set_default_settings(&s); s.time_limit = 1.0;
  return osqp_setup(&w, &d, &s); }" OSQP_COMPATIBLE_API)
if(NOT OSQP_COMPATIBLE_API)
    message(FATAL_ERROR "OSQP 0.6.x double-precision C API with PROFILING is required (validated: 0.6.3)")
endif()
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_LIBRARIES)

# 생성 C 소스를 배포한다. 일반 빌드/실행에는 CasADi나 Python이 필요하지 않다.
add_library(tamols_kernels SHARED generated/tamols_kernels.c)
target_include_directories(tamols_kernels PUBLIC generated)
target_link_libraries(tamols_kernels PRIVATE m)
add_library(tamols_sqp SHARED src/planner.cpp src/terrain.cpp src/robot_model.cpp src/dense_svd.cpp)
target_include_directories(tamols_sqp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> PRIVATE ${OSQP_INCLUDE_DIR} ${RBDL_INCLUDE_DIR})
target_link_libraries(tamols_sqp PUBLIC Eigen3::Eigen PRIVATE tamols_kernels ${OSQP_LIBRARY} ${LAPACK_LIBRARIES}
    ${RBDL_LIBRARY} ${RBDL_URDFREADER_LIBRARY} PkgConfig::TINYXML2)
add_executable(tamols_solve src/solve_main.cpp)
target_link_libraries(tamols_solve PRIVATE tamols_sqp yaml-cpp)
target_compile_definitions(tamols_solve PRIVATE
    B2_BUILD_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model/unitree_b2/b2.urdf")
set_target_properties(tamols_solve PROPERTIES INSTALL_RPATH "$ORIGIN/..")
set_target_properties(tamols_sqp PROPERTIES INSTALL_RPATH "$ORIGIN")
install(TARGETS tamols_sqp tamols_kernels LIBRARY DESTINATION lib)
install(TARGETS tamols_solve DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config model DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS
    PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/elevation_input_adapter.py scripts/elevation_mapping_runner.py
    DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/elevation_common.py DESTINATION lib/${PROJECT_NAME})

if(NOT B2_TERRAIN_STANDALONE)
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(grid_map_msgs REQUIRED)
    find_package(visualization_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)

    rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/LocomotionPlan.msg"
        DEPENDENCIES std_msgs geometry_msgs)

    # 지형 플래너 ROS 노드: elevation map + odom 구독 → Solve → LocomotionPlan 발행.
    add_executable(terrain_planner_node src/terrain_planner_node.cpp)
    target_compile_definitions(terrain_planner_node PRIVATE
        B2_BUILD_MODEL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/model/unitree_b2/b2.urdf")
    ament_target_dependencies(terrain_planner_node
        rclcpp std_msgs geometry_msgs nav_msgs grid_map_msgs visualization_msgs)
    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
    target_link_libraries(terrain_planner_node tamols_sqp ${cpp_typesupport_target})
    set_target_properties(terrain_planner_node PROPERTIES INSTALL_RPATH "$ORIGIN/..")
    install(TARGETS terrain_planner_node DESTINATION lib/${PROJECT_NAME})

    ament_export_include_directories(include)
    ament_export_libraries(tamols_sqp)
    ament_export_dependencies(Eigen3 rosidl_default_runtime)
    ament_package()
endif()
