.ONESHELL:
SHELL := /bin/bash
.DEFAULT_GOAL := build

.PHONY: clean
clean:
	@rm -rf build/ install/ log/ logs/ tests/

.PHONY: vcs-import
vcs-import:
	@VCS_FILE="${VCS_FILE}"
	vcs import < ${VCS_FILE}

.PHONY: rosdep-install
rosdep-install:
	source ./source_all.sh
	sudo apt update
	rosdep update
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src
	source ./source_all.sh

.PHONY: rosdep-install-eol
rosdep-install-eol:
	source ./source_all.sh
	sudo apt update
	rosdep update --include-eol-distros
	rosdep install -y -r --rosdistro ${ROS_DISTRO} --ignore-src --from-paths src
	source ./source_all.sh

.PHONY: svl
svl:
	source ./source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to simple_racing autonomy_launch keyboard_controller competition_timer
	source ./source_all.sh

.PHONY: build-select
build-select:
	@PACKAGES="${PACKAGES}"
	source ./source_all.sh
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ${PACKAGES}
	source ./source_all.sh