# walk_controller_mongodb/Makefile 수정
# Copyright 1996-2020 Cyberbotics Ltd.

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH = $(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
RESOURCES_PATH = $(WEBOTS_HOME)/projects/robots/robotis/darwin-op

INCLUDE = \
  -I"$(RESOURCES_PATH)/libraries/managers/include" \
  -I"$(RESOURCES_PATH)/libraries/robotis-op2/robotis/Framework/include"

LIBRARIES = \
  -L"$(RESOURCES_PATH)/libraries/robotis-op2" -lrobotis-op2 \
  -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers

# Windows-specific includes and libraries (for vcpkg and curl)
ifeq ($(OS),Windows_NT)
    INCLUDE += -I"C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows/include"
    LIBRARIES += -L"C:/Users/VICTUS/Desktop/vcpkg/installed/x64-windows/lib" -lcurl -lws2_32 -lwsock32
else
    LIBRARIES += -lpthread -lcurl
endif

CXX_SOURCES = $(wildcard *.cpp)

### Do not modify: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include
