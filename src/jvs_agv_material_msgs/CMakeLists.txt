cmake_minimum_required(VERSION 3.5)
project(jvs_agv_material_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MaterialSpec.msg"
  "msg/MaterialActual.msg"
  "action/LoadMaterials.action"
  "action/UnloadMaterials.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
