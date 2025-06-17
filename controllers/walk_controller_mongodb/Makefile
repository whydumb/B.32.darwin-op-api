# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Webots Makefile system with MongoDB support
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

# DarwinOP project path configuration
DARWINOP_PROJECT_PATH = C:/Users/VICTUS/Desktop/dawin_op/darwinop_narmal
RESOURCES_PATH = $(DARWINOP_PROJECT_PATH)

# MongoDB configuration (vcpkg based)
VCPKG_ROOT = C:/Users/VICTUS/Desktop/vcpkg
MONGODB_INCLUDE_PATH = $(VCPKG_ROOT)/installed/x64-windows/include
MONGODB_LIB_PATH = $(VCPKG_ROOT)/installed/x64-windows/lib

# Default Webots include paths
INCLUDE = -I"$(RESOURCES_PATH)/libraries/managers/include" \
          -I"$(RESOURCES_PATH)/libraries/robotis-op2/robotis/Framework/include"

# Add MongoDB include paths
INCLUDE += -I"$(MONGODB_INCLUDE_PATH)/mongocxx/v_noabi" \
           -I"$(MONGODB_INCLUDE_PATH)/bsoncxx/v_noabi" \
           -I"$(MONGODB_INCLUDE_PATH)"

# Library configuration
LIBRARIES = -L"$(RESOURCES_PATH)/libraries/robotis-op2" -lrobotis-op2 \
            -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers

# Add MongoDB libraries
LIBRARIES += -L"$(MONGODB_LIB_PATH)" \
             -lmongocxx \
             -lbsoncxx

# Windows system libraries
LIBRARIES += -lws2_32

# Compile definitions
CPPFLAGS += -DUSE_MONGODB \
            -D_USE_MATH_DEFINES \
            -DNOMINMAX \
            -D_CRT_SECURE_NO_WARNINGS

# C++ standard configuration - GCC/MinGW 플래그 사용
CXXFLAGS += -std=c++17 -Wall -Wextra

# Source files
CXX_SOURCES = $(wildcard *.cpp)

### Do not modify: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
