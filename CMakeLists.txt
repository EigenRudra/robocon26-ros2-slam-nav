cmake_minimum_required(VERSION 3.8)
project(robocon26_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 1. Install Launch Files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install Configuration Files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 3. Install World Files
install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install Maps
install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

# 5. Install the Script
install(PROGRAMS scripts/fix_robot.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
