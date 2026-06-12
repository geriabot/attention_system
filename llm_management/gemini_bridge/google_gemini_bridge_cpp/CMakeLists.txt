cmake_minimum_required(VERSION 3.8)
project(google_gemini_bridge_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gemini_bridge_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  gemini_bridge_interfaces
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

set(CURLPP_VENDOR_DIR ${PROJECT_SOURCE_DIR}/third_party/curlpp)
set(CPP_BASE64_VENDOR_DIR ${PROJECT_SOURCE_DIR}/third_party/cpp-base64)

if(EXISTS ${CURLPP_VENDOR_DIR}/CMakeLists.txt)
  set(CURLPP_BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared curlpp library" FORCE)
  set(CURLPP_BUILD_MODULES OFF CACHE BOOL "Build curlpp C++ module" FORCE)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
  add_subdirectory(${CURLPP_VENDOR_DIR} EXCLUDE_FROM_ALL)
  set(CURLPP_TARGET curlpp_static)
  set(CURLPP_INCLUDE_DIRS ${CURLPP_VENDOR_DIR}/include)
  set_target_properties(${CURLPP_TARGET} PROPERTIES POSITION_INDEPENDENT_CODE ON)
else()
  find_path(CURLPP_INCLUDE_DIR curlpp/cURLpp.hpp
    PATHS /usr/local/include
    NO_DEFAULT_PATH
  )
  find_library(CURLPP_LIBRARY curlpp
    PATHS /usr/local/lib
    NO_DEFAULT_PATH
  )
  if(NOT CURLPP_INCLUDE_DIR OR NOT CURLPP_LIBRARY)
    message(FATAL_ERROR
      "No se encontro curlpp. Ejecuta vcs import desde google_gemini_bridge_cpp "
      "usando thirdparty.repos o instala curlpp en /usr/local.")
  endif()
  set(CURLPP_TARGET ${CURLPP_LIBRARY})
  set(CURLPP_INCLUDE_DIRS ${CURLPP_INCLUDE_DIR})
endif()

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/GeminiBridge.cpp
  ${CPP_BASE64_VENDOR_DIR}/base64.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
  PRIVATE
    ${CPP_BASE64_VENDOR_DIR}
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PRIVATE
    ${CURLPP_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${CURLPP_TARGET}
)

add_executable(gemini_bridge_node src/gemini_bridge_cpp_node.cpp)
ament_target_dependencies(gemini_bridge_node ${dependencies})
target_link_libraries(gemini_bridge_node ${PROJECT_NAME})

install(TARGETS
  gemini_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
)

#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

install(
  DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
