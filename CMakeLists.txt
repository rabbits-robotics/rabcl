cmake_minimum_required(VERSION 3.8)

# ESP-IDF のコンポーネントとして（ファームの components/rabcl に submodule で置いて）ビルドする場合
if(ESP_PLATFORM)
  idf_component_register(
    SRCS
      src/component/bno055.cpp
      src/component/jga25_370.cpp
      src/component/ld_20mg.cpp
      src/controller/body_posture.cpp
      src/controller/leg_damper.cpp
      src/controller/omni_drive.cpp
      src/controller/pd_gravity_ff.cpp
      src/interface/can.cpp
      src/interface/uart.cpp
      src/protocol/parser.cpp
      src/protocol/protocol.cpp
      src/utils/utils.cpp
    INCLUDE_DIRS include)
  return()
endif()

project(rabcl)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(rabcl SHARED
  src/component/jga25_370.cpp
  src/component/ld_20mg.cpp
  src/controller/body_posture.cpp
  src/controller/leg_damper.cpp
  src/controller/omni_drive.cpp
  src/controller/pd_gravity_ff.cpp
  src/interface/uart.cpp
  src/interface/can.cpp
  src/protocol/parser.cpp
  src/protocol/protocol.cpp
  src/utils/utils.cpp
)

target_include_directories(rabcl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS
  rabcl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
ament_export_libraries(rabcl)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_utils test/test_utils.cpp)
  target_link_libraries(test_utils rabcl)

  ament_add_gtest(test_omni_drive test/test_omni_drive.cpp)
  target_link_libraries(test_omni_drive rabcl)

  ament_add_gtest(test_uart test/test_uart.cpp)
  target_link_libraries(test_uart rabcl)

  ament_add_gtest(test_can test/test_can.cpp)
  target_link_libraries(test_can rabcl)

  ament_add_gtest(test_jga25_370 test/test_jga25_370.cpp)
  target_link_libraries(test_jga25_370 rabcl)

  ament_add_gtest(test_ld_20mg test/test_ld_20mg.cpp)
  target_link_libraries(test_ld_20mg rabcl)

  ament_add_gtest(test_pd_gravity_ff test/test_pd_gravity_ff.cpp)
  target_link_libraries(test_pd_gravity_ff rabcl)

  ament_add_gtest(test_protocol test/test_protocol.cpp)
  target_link_libraries(test_protocol rabcl)

  ament_add_gtest(test_body_posture test/test_body_posture.cpp)
  target_link_libraries(test_body_posture rabcl)

  ament_add_gtest(test_leg_damper test/test_leg_damper.cpp)
  target_link_libraries(test_leg_damper rabcl)
endif()

ament_package()
