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

TARGETS = \
  walk_controller_mongodb.MakefileOP2

.PHONY: release debug profile clean

release debug profile clean: $(TARGETS)

%.Makefile:
	+@echo "# make" $(MAKECMDGOALS) $(@:.Makefile=)
	+@make -s -C $(@:.Makefile=) $(MAKECMDGOALS)

%.MakefileOP2:
	+@echo "# make" $(MAKECMDGOALS) $(@:.MakefileOP2=)
	+@make -s -C $(@:.MakefileOP2=) $(MAKECMDGOALS)
ifneq ($(MAKECMDGOALS),clean)
	+@cp -f Makefile.robotis-op2 $(@:.MakefileOP2=)/
else
	+@rm -f $(@:.MakefileOP2=)/Makefile.robotis-op2
endif