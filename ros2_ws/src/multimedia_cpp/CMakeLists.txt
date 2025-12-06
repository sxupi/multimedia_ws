cmake_minimum_required(VERSION 3.8)
project(multimedia_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Public headers
include_directories(
  include
)

########### LCD LIBRARY (oniongarlic/liquidcrystal-i2c) ###########
# Files are vendored into external/liquidcrystal-i2c

add_library(liquidcrystal_i2c
  external/liquidcrystal-i2c/I2CIO.cpp
  external/liquidcrystal-i2c/LCD.cpp
  external/liquidcrystal-i2c/LiquidCrystal.cpp
  external/liquidcrystal-i2c/LiquidCrystal_I2C.cpp
)

target_include_directories(liquidcrystal_i2c PUBLIC
  external/liquidcrystal-i2c
)

########### LCD 1602 NODE ###########

add_executable(lcd_1602_node
  src/lcd_1602/lcd_1602.cpp
  src/lcd_1602/lcd_1602_node.cpp
)
target_link_libraries(lcd_1602_node
  liquidcrystal_i2c
)
ament_target_dependencies(lcd_1602_node rclcpp std_msgs)

########### IR RECEIVER NODE ###########

add_executable(ir_receiver_node
  src/ir_receiver/ir_receiver_node.cpp
)
ament_target_dependencies(ir_receiver_node rclcpp std_msgs)

########### SI470X FM RECEIVER NODE ###########

add_executable(si470x_node
  src/si470x/si470x.cpp
  src/si470x/si470x_node.cpp
)
# link against libgpiod for GPIO control
target_link_libraries(si470x_node
  gpiod
)
ament_target_dependencies(si470x_node rclcpp std_msgs)

########### INSTALL ###########

install(TARGETS
  lcd_1602_node
  ir_receiver_node
  si470x_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

########### TESTING ###########

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()