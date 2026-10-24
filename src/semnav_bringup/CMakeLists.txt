cmake_minimum_required(VERSION 3.8)
project(semnav_bringup)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

# 世界文件与语义真值清单在**构建期生成**，同一份 apartment.yaml 同时产出两者：
# 世界和真值必须同源，否则 Phase 2 的「地图位置 vs 真值误差」评测失去意义。
# 生成器内置布局校验（插墙 / 越界 / 重叠 / 门洞净空），校验失败即构建失败。
set(GEN_WORLD ${CMAKE_CURRENT_BINARY_DIR}/worlds/apartment.world)
set(GEN_TRUTH ${CMAKE_CURRENT_BINARY_DIR}/config/apartment_truth.json)
add_custom_command(
  OUTPUT ${GEN_WORLD} ${GEN_TRUTH}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py
          --layout ${CMAKE_CURRENT_SOURCE_DIR}/config/apartment.yaml
          --out-world ${GEN_WORLD}
          --out-truth ${GEN_TRUTH}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_world.py
          ${CMAKE_CURRENT_SOURCE_DIR}/config/apartment.yaml
  COMMENT "生成住宅世界与语义真值清单（含布局校验）"
  VERBATIM
)
add_custom_target(semnav_world_gen ALL DEPENDS ${GEN_WORLD} ${GEN_TRUTH})

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME}
        OPTIONAL)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config DESTINATION share/${PROJECT_NAME})
# Keep the explicitly reviewed 3x world available alongside the generated
# baseline world.  It is derived from apartment_3x.yaml and checked before
# launch, so users can open it directly in Gazebo.
install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/worlds/apartment_3x.world
        ${CMAKE_CURRENT_SOURCE_DIR}/worlds/apartment_reference_3x.world
        DESTINATION share/${PROJECT_NAME}/worlds)
install(FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/config/apartment_3x.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/apartment_3x_truth.json
        ${CMAKE_CURRENT_SOURCE_DIR}/config/apartment_reference_3x.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/apartment_reference_3x_truth.json
        DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS scripts/generate_world.py scripts/setup_env.sh
                 scripts/phase1_mapping.py scripts/wait_ready.py
                 scripts/evaluate_map.py scripts/phase1_navigation.py
                 scripts/set_initial_pose.py scripts/check_reachability.py
                 scripts/check_tour.py scripts/phase1_nav_v9.py
                 scripts/semantic_locator.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
