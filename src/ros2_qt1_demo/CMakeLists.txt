cmake_minimum_required(VERSION 3.8)
project(ros2_qt1_demo)

# --------------------------------------------------------------------------
# 1. CẤU HÌNH QT TỰ ĐỘNG (Lấy từ bản QtCreator)
# --------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# --------------------------------------------------------------------------
# 2. TÌM PHỤ THUỘC (Dependencies)
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# find_package(ethercat_msgs REQUIRED)
# find_package(rclcpp REQUIRED) 

# --------------------------------------------------------------------------
# 3. QUẢN LÝ FILE NGUỒN (Lấy từ bản ROS 2)
# --------------------------------------------------------------------------
file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
file(GLOB_RECURSE UIS "ui/*.ui")
file(GLOB_RECURSE RESOURCES "resource/*.qrc")

# --------------------------------------------------------------------------
# 4. TẠO FILE THỰC THI (Executable)
# --------------------------------------------------------------------------

# Chỉ cho CMake biết các file .ui nằm trong thư mục ui/
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui")
add_executable(${PROJECT_NAME}
  src/main.cpp 
  ${SOURCES}
  ${HEADERS}
  ${UIS}
  ${RESOURCES}
)

# Thư mục include cho ROS 2
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Liên kết thư viện Qt
target_link_libraries(${PROJECT_NAME} PRIVATE
  Qt5::Widgets
  # rclcpp::rclcpp
)

# --------------------------------------------------------------------------
# 5. CẤU HÌNH CÀI ĐẶT & KẾT THÚC 
# --------------------------------------------------------------------------
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()