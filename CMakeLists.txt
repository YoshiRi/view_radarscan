cmake_minimum_required(VERSION 3.5)
project("view_radarscan")

# look for dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# install python scripts
# ament_cmake_install_package(${PROJECT_NAME})
ament_python_install_package(${PROJECT_NAME})

# install all python programs under the view_radarscan directory
file(GLOB SCRIPTS_FILES
RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
"${CMAKE_CURRENT_SOURCE_DIR}/view_radarscan/*.py")
foreach(SCRIPTS_FILE ${SCRIPTS_FILES})
  message(STATUS "Installing python programs: ${SCRIPTS_FILE}")
  install(PROGRAMS ${SCRIPTS_FILE}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# パッケージの設定を完了
ament_package()
