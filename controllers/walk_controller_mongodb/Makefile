# MongoDB 연동이 추가된 Webots Makefile (vcpkg 버전)
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
RESOURCES_PATH = $(WEBOTS_HOME)/projects/robots/robotis/darwin-op

# 기본 Webots 라이브러리
INCLUDE = -I"$(RESOURCES_PATH)/libraries/managers/include" -I"$(RESOURCES_PATH)/libraries/robotis-op2/robotis/Framework/include"
LIBRARIES = -L"$(RESOURCES_PATH)/libraries/robotis-op2" -lrobotis-op2 -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers

# vcpkg MongoDB 설정
VCPKG_ROOT = C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows

# vcpkg MongoDB 경로 확인 및 설정
ifneq ($(wildcard $(VCPKG_ROOT)/include/mongocxx),)
    # vcpkg MongoDB 헤더 경로
    INCLUDE += -I"$(VCPKG_ROOT)/include"
    
    # vcpkg MongoDB 라이브러리 경로
    LIBRARIES += -L"$(VCPKG_ROOT)/lib"
    
    # MongoDB 라이브러리들 (순서 중요!)
    LIBRARIES += -lmongocxx -lbsoncxx -lmongoc-1.0 -lbson-1.0
    
    # vcpkg의 의존성 라이브러리들
    LIBRARIES += -lssl -lcrypto -lcrypt32 -lws2_32 -ldnsapi -lsecur32
    
    # SASL2 (vcpkg에서 설치된 경우)
    ifneq ($(wildcard $(VCPKG_ROOT)/lib/sasl2.lib),)
        LIBRARIES += -lsasl2
    endif
    
    MONGODB_FOUND = yes
    MONGODB_SOURCE = vcpkg
else
    # 시스템 MongoDB 시도
    ifeq ($(OS),Windows_NT)
        MONGO_ROOT = C:/msys64/mingw64
        ifneq ($(wildcard $(MONGO_ROOT)/include/mongocxx),)
            INCLUDE += -I"$(MONGO_ROOT)/include/mongocxx/v_noabi" \
                       -I"$(MONGO_ROOT)/include/bsoncxx/v_noabi" \
                       -I"$(MONGO_ROOT)/include/libmongoc-1.0" \
                       -I"$(MONGO_ROOT)/include/libbson-1.0"
            LIBRARIES += -L"$(MONGO_ROOT)/lib" -lmongocxx -lbsoncxx -lmongoc-1.0 -lbson-1.0
            LIBRARIES += -lssl -lcrypto -lcrypt32 -lws2_32 -ldnsapi -lsecur32 -lsasl2
            MONGODB_FOUND = yes
            MONGODB_SOURCE = system
        endif
    endif
endif

# MongoDB를 찾지 못한 경우
ifndef MONGODB_FOUND
    MONGODB_SOURCE = not_found
endif

# C++ 표준 설정
CXXFLAGS += -std=c++17

# 소스 파일들
CXX_SOURCES = walk_controller_mongodb.cpp main.cpp

# MongoDB 확인 타겟
.PHONY: check-mongodb
check-mongodb:
	@echo "Checking MongoDB setup..."
	@echo "MongoDB source: $(MONGODB_SOURCE)"
ifeq ($(MONGODB_SOURCE),vcpkg)
	@echo "✅ Using vcpkg MongoDB at: $(VCPKG_ROOT)"
	@echo "Headers: $(VCPKG_ROOT)/include/mongocxx"
	@echo "Libraries: $(VCPKG_ROOT)/lib"
else ifeq ($(MONGODB_SOURCE),system)
	@echo "✅ Using system MongoDB"
else
	@echo "❌ MongoDB not found!"
	@echo ""
	@echo "Please install MongoDB using one of these methods:"
	@echo "1. vcpkg (recommended):"
	@echo "   cd C:/Users/VICTUS/Desktop/vcpkg"
	@echo "   ./vcpkg install mongo-cxx-driver:x64-windows"
	@echo ""
	@echo "2. MSYS2:"
	@echo "   pacman -S mingw-w64-x86_64-mongo-cxx-driver"
	@exit 1
endif

# vcpkg 정보 확인
.PHONY: check-vcpkg
check-vcpkg:
	@echo "Checking vcpkg installation..."
	@echo "vcpkg root: $(VCPKG_ROOT)"
	@if [ -d "$(VCPKG_ROOT)" ]; then \
		echo "✅ vcpkg directory found"; \
		if [ -d "$(VCPKG_ROOT)/include/mongocxx" ]; then \
			echo "✅ MongoDB headers found"; \
		else \
			echo "❌ MongoDB headers not found"; \
			echo "Install with: ./vcpkg install mongo-cxx-driver:x64-windows"; \
		fi; \
		if [ -f "$(VCPKG_ROOT)/lib/mongocxx.lib" ]; then \
			echo "✅ MongoDB libraries found"; \
		else \
			echo "❌ MongoDB libraries not found"; \
		fi; \
	else \
		echo "❌ vcpkg directory not found at $(VCPKG_ROOT)"; \
	fi

# 도움말
.PHONY: help
help:
	@echo "Available targets:"
	@echo "  all           - Build the controller"
	@echo "  clean         - Clean build files"
	@echo "  check-mongodb - Check MongoDB installation"
	@echo "  check-vcpkg   - Check vcpkg installation"
	@echo "  help          - Show this help"
	@echo ""
	@echo "MongoDB source: $(MONGODB_SOURCE)"

# 빌드 전 MongoDB 확인
all: check-mongodb

### Do not modify: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
