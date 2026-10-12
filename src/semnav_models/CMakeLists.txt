cmake_minimum_required(VERSION 3.8)
project(semnav_models)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# 模型在**构建期生成**，而不是把生成物提交进仓库：
#   1. 尺寸定义只有一处（scripts/generate_models.py），不会与 SDF 失配；
#   2. 模型集变更立刻反映到构建产物，不存在「忘了重新生成」。
set(GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/models)
add_custom_command(
  OUTPUT ${GEN_DIR}/models_manifest.json
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_models.py
          --out-dir ${GEN_DIR}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_models.py
  COMMENT "生成 semnav 语义家具模型集"
  VERBATIM
)
add_custom_target(semnav_models_gen ALL DEPENDS ${GEN_DIR}/models_manifest.json)

install(DIRECTORY ${GEN_DIR}/ DESTINATION share/${PROJECT_NAME}/models)
install(PROGRAMS scripts/generate_models.py DESTINATION lib/${PROJECT_NAME})

ament_package()
