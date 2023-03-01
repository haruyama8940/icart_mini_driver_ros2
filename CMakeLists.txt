cmake_minimum_required(VERSION 3.5)
project(icart_mini_driver)


find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

 
# ament_auto_add_library(${PROJECT_NAME} SHARED
#  src/icart_mini_driver_node.cpp)

# add_executable(icart_mini_driver src/icart_mini_driver_node.cpp)
ament_auto_add_executable(icart_mini_driver
          src/icart_mini_driver_node.cpp)

target_link_libraries(icart_mini_driver)

target_include_directories(icart_mini_driver
       PUBLIC ${ypspur_INCLUDE_DIRS})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
