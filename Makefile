default: build
.PHONY: build

define usage
[TARGETS]:
  deps:
    Install dependencies.

  submodules:
    Instanciate git submodules.

  build:
    Build this workspace.

  clean:
    Clean this workspace.
endef
export usage

help:
	@echo "$$usage"

.catkin_tools:
	@echo "[Instanciating catkin workspace]"
	@catkin init

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

submodules:
	@echo "[Instanciating git submodules]"
	@git submodule init
	@git submodule update

orbslam3:
	@bash ./scripts/build_orbslam3.bash

build: .catkin_tools orbslam3
	@echo "[Building]"
	@. /opt/ros/melodic/setup.sh && @catkin build -j2 -DCMAKE_BUILD_TYPE=Release

clean:
	@echo "[Cleaning]"
	@catkin clean
