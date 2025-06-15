cmake_minimum_required(VERSION 3.16)
project(walk_controller)

set(CMAKE_CXX_STANDARD 17)

# Webots 경로
if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME "C:/Users/VICTUS/AppData/Local/Programs/Webots")
endif()

# ROBOTIS 경로
set(ROBOTIS_PATH "${WEBOTS_HOME}/projects/robots/robotis/darwin-op")

# 소스 파일
file(GLOB CXX_SOURCES "*.cpp")
add_executable(${PROJECT_NAME} ${CXX_SOURCES})

# 헤더 경로
target_include_directories(${PROJECT_NAME} PRIVATE
    ${WEBOTS_HOME}/include/controller/cpp
    ${WEBOTS_HOME}/include/controller/c
    ${ROBOTIS_PATH}/libraries/managers/include
    ${ROBOTIS_PATH}/libraries/robotis-op2/robotis/Framework/include
)

# 모든 라이브러리
target_link_libraries(${PROJECT_NAME} PRIVATE
    ${WEBOTS_HOME}/lib/controller/CppController.lib
    ${WEBOTS_HOME}/lib/controller/Controller.lib
    ${ROBOTIS_PATH}/libraries/robotis-op2/robotis-op2.lib
    ${ROBOTIS_PATH}/libraries/managers/managers.lib
    mongo::mongocxx_shared
    mongo::bsoncxx_shared
    ws2_32 user32 gdi32 winmm
)

# MongoDB 설정
find_package(mongocxx CONFIG REQUIRED)
find_package(bsoncxx CONFIG REQUIRED)
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_MONGODB)

# 출력 경로
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}
)
