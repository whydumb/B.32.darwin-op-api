cmake_minimum_required(VERSION 3.15)
project(walk_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 디버깅: 경로 출력
message(STATUS "========== CMake Configuration ==========")
message(STATUS "CMAKE_TOOLCHAIN_FILE: ${CMAKE_TOOLCHAIN_FILE}")
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
message(STATUS "VCPKG_TARGET_TRIPLET: ${VCPKG_TARGET_TRIPLET}")

# vcpkg 경로 자동 감지 및 설정
if(NOT CMAKE_TOOLCHAIN_FILE AND EXISTS "C:/Users/VICTUS/Desktop/vcpkg/scripts/buildsystems/vcpkg.cmake")
    set(CMAKE_TOOLCHAIN_FILE "C:/Users/VICTUS/Desktop/vcpkg/scripts/buildsystems/vcpkg.cmake")
    message(STATUS "vcpkg 툴체인 자동 설정: ${CMAKE_TOOLCHAIN_FILE}")
endif()

# 수동으로 vcpkg 경로 설정 (백업용)
if(EXISTS "C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows")
    set(CMAKE_PREFIX_PATH "C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows" ${CMAKE_PREFIX_PATH})
    message(STATUS "vcpkg 수동 경로 추가됨")
endif()

# Webots 경로
if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME "C:/Users/VICTUS/AppData/Local/Programs/Webots")
endif()

# ROBOTIS 경로
set(ROBOTIS_PATH "${WEBOTS_HOME}/projects/robots/robotis/darwin-op")

message(STATUS "Webots path: ${WEBOTS_HOME}")
message(STATUS "ROBOTIS path: ${ROBOTIS_PATH}")

# 경로 존재 확인
if(NOT EXISTS "${WEBOTS_HOME}")
    message(WARNING "Webots 경로를 찾을 수 없습니다: ${WEBOTS_HOME}")
endif()

if(NOT EXISTS "${ROBOTIS_PATH}")
    message(WARNING "ROBOTIS 경로를 찾을 수 없습니다: ${ROBOTIS_PATH}")
endif()

# MongoDB 드라이버 찾기
message(STATUS "========== MongoDB Configuration ==========")
find_package(bsoncxx CONFIG REQUIRED)
find_package(mongocxx CONFIG REQUIRED)

message(STATUS "bsoncxx found: ${bsoncxx_FOUND}")
message(STATUS "mongocxx found: ${mongocxx_FOUND}")

# 어떤 MongoDB 타겟이 사용 가능한지 확인
if(TARGET mongo::bsoncxx_static)
    message(STATUS "bsoncxx static 라이브러리 사용 가능")
    set(BSONCXX_TARGET mongo::bsoncxx_static)
elseif(TARGET mongo::bsoncxx_shared)
    message(STATUS "bsoncxx shared 라이브러리 사용")
    set(BSONCXX_TARGET mongo::bsoncxx_shared)
else()
    message(FATAL_ERROR "bsoncxx 라이브러리를 찾을 수 없습니다!")
endif()

if(TARGET mongo::mongocxx_static)
    message(STATUS "mongocxx static 라이브러리 사용 가능")
    set(MONGOCXX_TARGET mongo::mongocxx_static)
elseif(TARGET mongo::mongocxx_shared)
    message(STATUS "mongocxx shared 라이브러리 사용")
    set(MONGOCXX_TARGET mongo::mongocxx_shared)
else()
    message(FATAL_ERROR "mongocxx 라이브러리를 찾을 수 없습니다!")
endif()

# 소스 파일
file(GLOB CXX_SOURCES "*.cpp")
message(STATUS "========== Source Files ==========")
foreach(source ${CXX_SOURCES})
    message(STATUS "소스 파일: ${source}")
endforeach()

add_executable(${PROJECT_NAME} ${CXX_SOURCES})

# MongoDB 활성화 (타겟 생성 후 바로 설정)
target_compile_definitions(${PROJECT_NAME} PRIVATE USE_MONGODB)

# 헤더 경로
target_include_directories(${PROJECT_NAME} PRIVATE
    ${WEBOTS_HOME}/include/controller/cpp
    ${WEBOTS_HOME}/include/controller/c
    ${ROBOTIS_PATH}/libraries/managers/include
    ${ROBOTIS_PATH}/libraries/robotis-op2/robotis/Framework/include
)

# 라이브러리 링크 순서 최적화
message(STATUS "========== Library Linking ==========")
target_link_libraries(${PROJECT_NAME} PRIVATE
    # MongoDB 라이브러리 (먼저 링크)
    ${MONGOCXX_TARGET}
    ${BSONCXX_TARGET}
    
    # Webots 라이브러리
    ${WEBOTS_HOME}/lib/controller/CppController.lib
    ${WEBOTS_HOME}/lib/controller/Controller.lib
    
    # ROBOTIS 라이브러리
    ${ROBOTIS_PATH}/libraries/robotis-op2/robotis-op2.lib
    ${ROBOTIS_PATH}/libraries/managers/managers.lib
    
    # Windows 시스템 라이브러리
    ws2_32 user32 gdi32 winmm
)

message(STATUS "MongoDB 타겟: ${MONGOCXX_TARGET}, ${BSONCXX_TARGET}")

# 컴파일러별 설정
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE 
        /W3          # 경고 레벨
        /bigobj      # 큰 객체 파일 지원
        /EHsc        # 예외 처리
    )
    
    # Release 모드에서 최적화
    target_compile_options(${PROJECT_NAME} PRIVATE 
        $<$<CONFIG:Release>:/O2>
    )
else()
    target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)
endif()

# 출력 경로
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}
)

# Windows에서 DLL 자동 복사
if(WIN32 AND TARGET mongo::mongocxx_shared)
    message(STATUS "Windows DLL 복사 설정")
    
    # MongoDB DLL 복사
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${MONGOCXX_TARGET}>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMENT "MongoDB mongocxx DLL 복사"
    )
    
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${BSONCXX_TARGET}>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMENT "MongoDB bsoncxx DLL 복사"
    )
endif()

# 빌드 후 정보 출력
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
    COMMAND ${CMAKE_COMMAND} -E echo "빌드 완료: ${PROJECT_NAME}"
    COMMAND ${CMAKE_COMMAND} -E echo "실행 파일: $<TARGET_FILE:${PROJECT_NAME}>"
    COMMAND ${CMAKE_COMMAND} -E echo "=========================================="
)

message(STATUS "========== Configuration Complete ==========")
