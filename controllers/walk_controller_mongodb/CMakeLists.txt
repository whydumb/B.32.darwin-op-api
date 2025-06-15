cmake_minimum_required(VERSION 3.16)

# 프로젝트 정의
project(walk_controller_mongodb)

# C++ 표준 설정 (C++로 변경!)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Webots 환경 변수 설정
if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME "C:/Users/VICTUS/AppData/Local/Programs/Webots")
endif()

message(STATUS "WEBOTS_HOME: ${WEBOTS_HOME}")

# Webots C++ API 라이브러리 경로 설정 (C++로 변경!)
set(WEBOTS_LIB_DIR "${WEBOTS_HOME}/lib/controller")
set(WEBOTS_INCLUDE_DIR "${WEBOTS_HOME}/include/controller/cpp")

# ROBOTIS OP2 라이브러리 경로 추가
set(ROBOTIS_PATH "${WEBOTS_HOME}/projects/robots/robotis/darwin-op")
set(ROBOTIS_LIBRARIES_PATH "${ROBOTIS_PATH}/libraries")
set(ROBOTIS_MANAGERS_INCLUDE "${ROBOTIS_LIBRARIES_PATH}/managers/include")
set(ROBOTIS_FRAMEWORK_INCLUDE "${ROBOTIS_LIBRARIES_PATH}/robotis-op2/robotis/Framework/include")

# Webots C++ 라이브러리 
set(WEBOTS_CONTROLLER_LIB "${WEBOTS_LIB_DIR}/Controller.lib")
set(WEBOTS_CPP_CONTROLLER_LIB "${WEBOTS_LIB_DIR}/CppController.lib")

# 파일 존재 확인
message(STATUS "Checking Webots C++ libraries:")
message(STATUS "  Controller lib: ${WEBOTS_CONTROLLER_LIB}")
message(STATUS "  CppController lib: ${WEBOTS_CPP_CONTROLLER_LIB}")

if(EXISTS ${WEBOTS_CONTROLLER_LIB} AND EXISTS ${WEBOTS_CPP_CONTROLLER_LIB})
    message(STATUS "    ✓ Found Webots C++ libraries")
    set(USE_WEBOTS_CONTROLLER TRUE)
else()
    message(STATUS "    ✗ Webots C++ libraries not found")
    set(USE_WEBOTS_CONTROLLER FALSE)
endif()

# ROBOTIS OP2 라이브러리 확인
set(ROBOTIS_MANAGERS_LIB "${ROBOTIS_LIBRARIES_PATH}/managers/managers.lib")
set(ROBOTIS_OP2_LIB "${ROBOTIS_LIBRARIES_PATH}/robotis-op2/robotis-op2.lib")

if(EXISTS ${ROBOTIS_MANAGERS_LIB} AND EXISTS ${ROBOTIS_OP2_LIB})
    message(STATUS "    ✓ Found ROBOTIS OP2 libraries")
    set(USE_ROBOTIS_OP2 TRUE)
else()
    message(STATUS "    ✗ ROBOTIS OP2 libraries not found")
    set(USE_ROBOTIS_OP2 FALSE)
endif()

# MongoDB C 라이브러리 찾기 (C API로 변경!)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
    pkg_check_modules(MONGOC QUIET libmongoc-1.0)
    pkg_check_modules(BSON QUIET libbson-1.0)
endif()

# vcpkg로 MongoDB C 드라이버 찾기
if(NOT MONGOC_FOUND)
    find_package(mongoc-1.0 CONFIG QUIET)
    find_package(bson-1.0 CONFIG QUIET)
    if(mongoc-1.0_FOUND AND bson-1.0_FOUND)
        set(MONGOC_FOUND TRUE)
        set(MONGOC_LIBRARIES mongo::mongoc_shared)
        set(BSON_LIBRARIES mongo::bson_shared)
    endif()
endif()

if(MONGOC_FOUND)
    message(STATUS "MongoDB C libraries found")
    set(USE_MONGODB TRUE)
else()
    message(WARNING "MongoDB C libraries not found, building without MongoDB support")
    message(WARNING "Install with: vcpkg install mongo-c-driver:x64-windows")
    set(USE_MONGODB FALSE)
endif()

# 소스 파일들 (main.cpp 추가!)
set(SOURCES
    main.cpp
    walk_controller_mongodb.cpp
)

# 실행 파일 생성
add_executable(${PROJECT_NAME} ${SOURCES})

# 헤더 파일 경로 추가
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${WEBOTS_INCLUDE_DIR}
    ${ROBOTIS_MANAGERS_INCLUDE}
    ${ROBOTIS_FRAMEWORK_INCLUDE}
)

# 컴파일 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<BOOL:${USE_MONGODB}>:USE_MONGODB>
    $<$<NOT:$<BOOL:${USE_MONGODB}>>:NO_MONGODB>
)

# Webots C++ 라이브러리 링크
if(USE_WEBOTS_CONTROLLER)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${WEBOTS_CONTROLLER_LIB}
        ${WEBOTS_CPP_CONTROLLER_LIB}
    )
    message(STATUS "Linked Webots C++ libraries")
endif()

# ROBOTIS OP2 라이브러리 링크
if(USE_ROBOTIS_OP2)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        ${ROBOTIS_MANAGERS_LIB}
        ${ROBOTIS_OP2_LIB}
    )
    message(STATUS "Linked ROBOTIS OP2 libraries")
endif()

# MongoDB C 라이브러리 (C API!)
if(USE_MONGODB)
    if(MONGOC_INCLUDE_DIRS)
        # pkg-config 방식
        target_include_directories(${PROJECT_NAME} PRIVATE ${MONGOC_INCLUDE_DIRS} ${BSON_INCLUDE_DIRS})
        target_link_directories(${PROJECT_NAME} PRIVATE ${MONGOC_LIBRARY_DIRS} ${BSON_LIBRARY_DIRS})
        target_link_libraries(${PROJECT_NAME} PRIVATE ${MONGOC_LIBRARIES} ${BSON_LIBRARIES})
    else()
        # vcpkg 방식
        target_link_libraries(${PROJECT_NAME} PRIVATE ${MONGOC_LIBRARIES} ${BSON_LIBRARIES})
    endif()
    message(STATUS "Linked MongoDB C libraries")
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
        secur32
        crypt32
        bcrypt
    )
endif()

# 디버그 정보
message(STATUS "=== Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "Webots C++ Controller: ${USE_WEBOTS_CONTROLLER}")
message(STATUS "ROBOTIS OP2: ${USE_ROBOTIS_OP2}")
message(STATUS "MongoDB Support: ${USE_MONGODB}")
message(STATUS "===========================")

# MSVC 컴파일러 플래그
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W3
        /utf-8
        /wd4996  # deprecated warnings
    )
endif()

# Windows 특정 설정
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()
