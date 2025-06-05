# Fixed Makefile for walk_controller_mongodb
# Copyright 1996-2020 Cyberbotics Ltd.

null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
RESOURCES_PATH = $(WEBOTS_HOME)/projects/robots/robotis/darwin-op
INCLUDE = -I"$(RESOURCES_PATH)/libraries/managers/include" -I"$(RESOURCES_PATH)/libraries/robotis-op2/robotis/Framework/include"

LIBRARIES = -L"$(RESOURCES_PATH)/libraries/robotis-op2" -lrobotis-op2 -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers

# Add Windows-specific libraries for socket programming
ifeq ($(OS),Windows_NT)
    LIBRARIES += -lws2_32 -lwsock32
else
    # For Linux/Unix systems
    LIBRARIES += -lpthread
endif

CXX_SOURCES = $(wildcard *.cpp)

### Do not modify: this includes Webots global Makefile.include
include $(WEBOTS_HOME_PATH)/resources/Makefile.include