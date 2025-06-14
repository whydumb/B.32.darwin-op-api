cmake_minimum_required(VERSION 3.16)

# 프로젝트 정의
project(walk_controller_mongodb)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots 환경 변수 설정
if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME "C:/Users/VICTUS/AppData/Local/Programs/Webots")
endif()

message(STATUS "WEBOTS_HOME: ${WEBOTS_HOME}")

# Webots 라이브러리 경로 설정
set(WEBOTS_LIB_DIR "${WEBOTS_HOME}/lib/controller")
set(WEBOTS_INCLUDE_DIR "${WEBOTS_HOME}/include/controller/cpp")

# 실제 존재하는 라이브러리 파일들만 사용
set(WEBOTS_CONTROLLER_LIB "${WEBOTS_LIB_DIR}/Controller.lib")
# CppController.lib는 존재하지 않으므로 제거

# 파일 존재 확인
message(STATUS "Checking Webots libraries:")
message(STATUS "  Controller lib: ${WEBOTS_CONTROLLER_LIB}")
if(EXISTS ${WEBOTS_CONTROLLER_LIB})
    message(STATUS "    ✓ Found")
    set(USE_WEBOTS_CONTROLLER TRUE)
else()
    message(STATUS "    ✗ Not found")
    set(USE_WEBOTS_CONTROLLER FALSE)
endif()

# MongoDB 라이브러리 찾기
find_package(mongocxx CONFIG)
find_package(bsoncxx CONFIG)

if(mongocxx_FOUND AND bsoncxx_FOUND)
    message(STATUS "MongoDB libraries found")
    set(USE_MONGODB TRUE)
else()
    message(WARNING "MongoDB libraries not found, building without MongoDB support")
    set(USE_MONGODB FALSE)
endif()

# 소스 파일들
set(SOURCES
    main.cpp
    walk_controller_mongodb.cpp
)

set(HEADERS
    walk_controller_mongodb.hpp
)

# 실행 파일 생성
add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# 헤더 파일 경로 추가
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${WEBOTS_INCLUDE_DIR}
)

# 컴파일 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<BOOL:${USE_MONGODB}>:USE_MONGODB>
)

# Webots 라이브러리 링크 (존재하는 것만)
if(USE_WEBOTS_CONTROLLER)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${WEBOTS_CONTROLLER_LIB}
    )
    message(STATUS "Linked Controller.lib")
endif()

# MongoDB 라이브러리 (선택적)
if(USE_MONGODB)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        $<IF:$<TARGET_EXISTS:mongo::mongocxx_static>,mongo::mongocxx_static,mongo::mongocxx_shared>
        $<IF:$<TARGET_EXISTS:mongo::bsoncxx_static>,mongo::bsoncxx_static,mongo::bsoncxx_shared>
    )
endif()

# Windows 시스템 라이브러리
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ws2_32
        user32
        gdi32
        winmm
        ole32
        oleaut32
    )
endif()

# 디버그 정보
message(STATUS "=== Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Webots Controller: ${USE_WEBOTS_CONTROLLER}")
message(STATUS "MongoDB Support: ${USE_MONGODB}")
message(STATUS "===========================")

# MSVC 컴파일러 플래그
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W3
        /utf-8
    )
endif()

# Windows 특정 설정
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}
    )
endif()
