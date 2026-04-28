cmake_minimum_required(VERSION 3.10)
project(world)

find_package(ament_cmake REQUIRED)

# === 월드 / 모델 / 런치 / 컨피그 (소스 트리에 그대로 있는 것들) ===
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# uav26_quad 모델: 그대로 복사
install(DIRECTORY models/uav26_quad
        DESTINATION share/${PROJECT_NAME}/models)

# world_assets 모델: model.config / model.sdf 만 복사
install(DIRECTORY models/world_assets
        DESTINATION share/${PROJECT_NAME}/models
        PATTERN "meshes" EXCLUDE
        PATTERN "textures" EXCLUDE)

# === world_assets 안으로 mesh / textures 를 install rule 로 주입 ===
# (소스 트리에서는 mesh/, textures/ 가 패키지 root 에 있고,
#  설치 후엔 share/world/models/world_assets/{meshes,textures}/ 에 위치)
install(DIRECTORY mesh/
        DESTINATION share/${PROJECT_NAME}/models/world_assets/meshes
        FILES_MATCHING PATTERN "*.stl")

install(DIRECTORY textures/
        DESTINATION share/${PROJECT_NAME}/models/world_assets/textures
        FILES_MATCHING PATTERN "*.png")

ament_package()
