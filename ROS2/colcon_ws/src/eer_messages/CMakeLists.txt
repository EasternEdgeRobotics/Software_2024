cmake_minimum_required(VERSION 3.8)
project(eer_messages)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/Config.srv"
	"srv/HSVColours.srv"
	"msg/ThrusterMultipliers.msg"
	"msg/PilotInput.msg"
	"msg/DiagnosticsData.msg"
	"msg/OutsideTempProbeData.msg"
	"action/AutoMode.action"
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
