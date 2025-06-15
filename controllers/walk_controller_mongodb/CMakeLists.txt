cmake_minimum_required(VERSION 3.16)

# 프로젝트 정의
project(walk_controller)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# MongoDB 지원 옵션 (기본값: OFF, 선택적 활성화)
option(ENABLE_MONGODB "Enable MongoDB support" OFF)

# Webots 환경 변수 설정
if(DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME "C:/Users/VICTUS/AppData/Local/Programs/Webots")
endif()

message(STATUS "WEBOTS_HOME: ${WEBOTS_HOME}")

# Webots C++ API 라이브러리 경로 설정
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

# MongoDB 지원 확인 (선택적)
set(USE_MONGODB FALSE)
if(ENABLE_MONGODB)
    find_package(mongocxx CONFIG QUIET)
    find_package(bsoncxx CONFIG QUIET)
    
    if(mongocxx_FOUND AND bsoncxx_FOUND)
        message(STATUS "    ✓ MongoDB C++ libraries found")
        set(USE_MONGODB TRUE)
    else()
        message(WARNING "MongoDB C++ libraries not found")
        message(WARNING "Install with: vcpkg install mongo-cxx-driver:x64-windows")
        message(WARNING "Or disable with: -DENABLE_MONGODB=OFF")
    endif()
else()
    message(STATUS "MongoDB support disabled (use -DENABLE_MONGODB=ON to enable)")
endif()

# 소스 파일들
set(SOURCES
    main.cpp
    walk_controller.cpp
)

# 실행 파일 생성
add_executable(${PROJECT_NAME} ${SOURCES})

# 기본 헤더 파일 경로
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${WEBOTS_INCLUDE_DIR}
    ${ROBOTIS_MANAGERS_INCLUDE}
    ${ROBOTIS_FRAMEWORK_INCLUDE}
)

# 기본 컴파일 정의
target_compile_definitions(${PROJECT_NAME} PRIVATE
    $<$<BOOL:${USE_MONGODB}>:USE_MONGODB>
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

# MongoDB C++ 라이브러리 링크 (조건부)
if(USE_MONGODB)
    target_link_libraries(${PROJECT_NAME} PRIVATE
        mongo::mongocxx_shared
        mongo::bsoncxx_shared
    )
    message(STATUS "Linked MongoDB C++ libraries")
    
    # MongoDB 관련 추가 설정
    target_compile_definitions(${PROJECT_NAME} PRIVATE
        MONGOCXX_STATIC_DEFINE
        BSONCXX_STATIC_DEFINE
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
        secur32
        crypt32
        bcrypt
    )
endif()

# 빌드 구성 정보
message(STATUS "=== Build Configuration ===")
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Webots Controller: ${USE_WEBOTS_CONTROLLER}")
message(STATUS "ROBOTIS OP2: ${USE_ROBOTIS_OP2}")
message(STATUS "MongoDB Support: ${USE_MONGODB}")
if(USE_MONGODB)
    message(STATUS "  MongoDB Features: Action save/load, Real-time control")
else()
    message(STATUS "  Fallback: Basic keyboard control only")
endif()
message(STATUS "===========================")

# 컴파일러별 설정
if(MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE
        /W3
        /utf8
        /wd4996  # deprecated warnings
        /wd4267  # size_t conversion warnings
        /wd4244  # type conversion warnings
        $<$<BOOL:${USE_MONGODB}>:/bigobj>  # MongoDB는 큰 오브젝트 파일 생성 가능
    )
endif()

# Windows 출력 디렉토리 설정
if(WIN32)
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_SOURCE_DIR}
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}
    )
endif()

# 빌드 타입별 최적화
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(${PROJECT_NAME} PRIVATE DEBUG_BUILD)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_definitions(${PROJECT_NAME} PRIVATE RELEASE_BUILD)
endif()
