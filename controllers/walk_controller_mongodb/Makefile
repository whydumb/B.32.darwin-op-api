# MongoDB 연동이 추가된 Webots Makefile (CMake 방식 반영)
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
RESOURCES_PATH = $(WEBOTS_HOME)/projects/robots/robotis/darwin-op

# 기본 Webots 라이브러리
INCLUDE = -I"$(RESOURCES_PATH)/libraries/managers/include" -I"$(RESOURCES_PATH)/libraries/robotis-op2/robotis/Framework/include"
LIBRARIES = -L"$(RESOURCES_PATH)/libraries/robotis-op2" -lrobotis-op2 -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers

# vcpkg MongoDB 설정 (CMake find_package 방식 모방)
VCPKG_ROOT = C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows

# MongoDB 헤더 및 라이브러리 검출
ifneq ($(wildcard $(VCPKG_ROOT)/include/mongocxx),)
    # MongoDB 헤더 경로들
    INCLUDE += -I"$(VCPKG_ROOT)/include"
    
    # 라이브러리 경로
    LIBRARIES += -L"$(VCPKG_ROOT)/lib"
    
    # CMake가 선택하는 방식과 동일하게: static 우선, 없으면 shared
    # 1. Static 라이브러리 확인
    MONGOCXX_STATIC_EXISTS := $(wildcard $(VCPKG_ROOT)/lib/*mongocxx*static*.lib)
    BSONCXX_STATIC_EXISTS := $(wildcard $(VCPKG_ROOT)/lib/*bsoncxx*static*.lib)
    
    # 2. Shared 라이브러리 확인  
    MONGOCXX_SHARED_EXISTS := $(wildcard $(VCPKG_ROOT)/lib/*mongocxx*.lib)
    BSONCXX_SHARED_EXISTS := $(wildcard $(VCPKG_ROOT)/lib/*bsoncxx*.lib)
    
    # 3. 실제 라이브러리 파일명 검출
    MONGOCXX_LIB_FILES := $(notdir $(basename $(wildcard $(VCPKG_ROOT)/lib/*mongocxx*.lib)))
    BSONCXX_LIB_FILES := $(notdir $(basename $(wildcard $(VCPKG_ROOT)/lib/*bsoncxx*.lib)))
    
    # 4. Static 우선 선택 (CMake 방식)
    ifneq ($(MONGOCXX_STATIC_EXISTS),)
        # Static 라이브러리 사용
        MONGOCXX_LIB := $(firstword $(filter %static,$(MONGOCXX_LIB_FILES)))
        BSONCXX_LIB := $(firstword $(filter %static,$(BSONCXX_LIB_FILES)))
        MONGODB_TYPE = static
        CXXFLAGS += -DMONGOCXX_STATIC -DBSONCXX_STATIC
    else ifneq ($(MONGOCXX_SHARED_EXISTS),)
        # Shared 라이브러리 사용
        MONGOCXX_LIB := $(firstword $(MONGOCXX_LIB_FILES))
        BSONCXX_LIB := $(firstword $(BSONCXX_LIB_FILES))
        MONGODB_TYPE = shared
    endif
    
    # 검출된 라이브러리가 있으면 링크
    ifneq ($(MONGOCXX_LIB),)
        LIBRARIES += -l$(MONGOCXX_LIB) -l$(BSONCXX_LIB)
        
        # MongoDB C 드라이버 (항상 필요)
        LIBRARIES += -lmongoc-1.0 -lbson-1.0
        
        # 시스템 의존성 라이브러리들
        LIBRARIES += -lws2_32 -lcrypt32 -lsecur32 -ldnsapi
        
        # vcpkg 의존성들 (있는 경우에만)
        # OpenSSL
        ifneq ($(wildcard $(VCPKG_ROOT)/lib/*ssl*.lib),)
            SSL_LIB := $(notdir $(basename $(firstword $(wildcard $(VCPKG_ROOT)/lib/*ssl*.lib))))
            CRYPTO_LIB := $(notdir $(basename $(firstword $(wildcard $(VCPKG_ROOT)/lib/*crypto*.lib))))
            LIBRARIES += -l$(SSL_LIB) -l$(CRYPTO_LIB)
        endif
        
        # zlib
        ifneq ($(wildcard $(VCPKG_ROOT)/lib/*zlib*.lib),)
            ZLIB_LIB := $(notdir $(basename $(firstword $(wildcard $(VCPKG_ROOT)/lib/*zlib*.lib))))
            LIBRARIES += -l$(ZLIB_LIB)
        endif
        
        # SASL2
        ifneq ($(wildcard $(VCPKG_ROOT)/lib/*sasl*.lib),)
            SASL_LIB := $(notdir $(basename $(firstword $(wildcard $(VCPKG_ROOT)/lib/*sasl*.lib))))
            LIBRARIES += -l$(SASL_LIB)
        endif
        
        MONGODB_FOUND = yes
        MONGODB_SOURCE = vcpkg
    endif
endif

# MongoDB를 찾지 못한 경우
ifndef MONGODB_FOUND
    MONGODB_SOURCE = not_found
    $(warning MongoDB not found - building without MongoDB support)
    # MongoDB 없는 기본 소스만 사용
    CXX_SOURCES = main.cpp walk_controller.cpp
else
    # MongoDB 있는 소스 사용
    CXX_SOURCES = walk_controller_mongodb.cpp main.cpp
endif

# C++ 표준 설정
CXXFLAGS += -std=c++17 -Wno-deprecated-declarations

# 라이브러리 검출 결과 출력
.PHONY: detect-mongodb-libs
detect-mongodb-libs:
	@echo "=== MongoDB 라이브러리 자동 검출 ==="
	@echo "검사 경로: $(VCPKG_ROOT)/lib"
	@echo ""
	@echo "검출된 MongoDB C++ 라이브러리 파일들:"
	@echo "MONGOCXX files: $(MONGOCXX_LIB_FILES)"
	@echo "BSONCXX files: $(BSONCXX_LIB_FILES)"
	@echo ""
	@echo "선택된 라이브러리:"
	@echo "MONGOCXX: $(MONGOCXX_LIB)"
	@echo "BSONCXX: $(BSONCXX_LIB)"
	@echo "Type: $(MONGODB_TYPE)"
	@echo ""
	@echo "실제 파일 존재 확인:"
	@ls -la "$(VCPKG_ROOT)/lib" 2>/dev/null | grep -E "(mongocxx|bsoncxx)" || echo "MongoDB 라이브러리 파일 없음"

# MongoDB 확인 타겟
.PHONY: check-mongodb
check-mongodb:
	@echo "Checking MongoDB setup..."
	@echo "MongoDB source: $(MONGODB_SOURCE)"
ifeq ($(MONGODB_SOURCE),vcpkg)
	@echo "✅ Using vcpkg MongoDB at: $(VCPKG_ROOT)"
	@echo "Selected libraries: $(MONGOCXX_LIB), $(BSONCXX_LIB)"
	@echo "Library type: $(MONGODB_TYPE)"
else
	@echo "❌ MongoDB not found!"
	@echo ""
	@echo "Please install MongoDB using vcpkg:"
	@echo "cd C:/Users/VICTUS/Desktop/vcpkg"
	@echo "./vcpkg install mongo-cxx-driver:x64-windows"
endif

# 빌드 정보 출력
.PHONY: build-info
build-info:
	@echo "=== Build Configuration ==="
	@echo "Source files: $(CXX_SOURCES)"
	@echo "MongoDB found: $(MONGODB_FOUND)"
ifdef MONGODB_FOUND
	@echo "MongoDB type: $(MONGODB_TYPE)"
	@echo "MongoDB libs: $(MONGOCXX_LIB) $(BSONCXX_LIB)"
endif
	@echo "Include paths: $(INCLUDE)"
	@echo "Libraries: $(LIBRARIES)"
	@echo "=========================="

# 상세 디버그 정보
.PHONY: debug-libs
debug-libs:
	@echo "=== 디버그 정보 ==="
	@echo "VCPKG_ROOT: $(VCPKG_ROOT)"
	@echo "Static exists: mongocxx=$(words $(MONGOCXX_STATIC_EXISTS)), bsoncxx=$(words $(BSONCXX_STATIC_EXISTS))"
	@echo "Shared exists: mongocxx=$(words $(MONGOCXX_SHARED_EXISTS)), bsoncxx=$(words $(BSONCXX_SHARED_EXISTS))"
	@echo "Static files: $(MONGOCXX_STATIC_EXISTS)"
	@echo "Shared files: $(MONGOCXX_SHARED_EXISTS)"
	@echo "All mongocxx files: $(MONGOCXX_LIB_FILES)"
	@echo "All bsoncxx files: $(BSONCXX_LIB_FILES)"

# 도움말
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all                  - Build the controller"
	@echo "  clean                - Clean build files"
	@echo "  check-mongodb        - Check MongoDB installation"
	@echo "  detect-mongodb-libs  - Auto-detect MongoDB libraries"
	@echo "  build-info          - Show build configuration"
	@echo "  debug-libs          - Show detailed library detection info"
	@echo "  help                - Show this help"

# 빌드 전 MongoDB 확인
all: build-info

### Do not modify: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
