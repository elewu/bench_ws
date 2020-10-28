default: repos build
.PHONY: repos build

.catkin_tools:
	@catkin init

repos:
	@git submodule init

build: .catkin_tools
	@catkin build -j2 -DCMAKE_BUILD_TYPE=Release
