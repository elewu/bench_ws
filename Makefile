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

build: .catkin_tools
	@echo "[Building]"
	@ln -fs ${PWD}/scripts/orbslam3_package.xml src/ORB_SLAM3/package.xml
	@. /opt/ros/melodic/setup.sh && catkin build -j2 -DCMAKE_BUILD_TYPE=Release

clean:
	@echo "[Cleaning]"
	@catkin clean
